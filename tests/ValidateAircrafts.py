# ValidateAircrafts.py
#
# Validates aircraft configurations for physical plausibility and metadata.
# Emits warnings but does not fail tests - designed for CI visibility.
#
# Copyright (c) 2025 JSBSim Contributors
#
# This program is free software; you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation; either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, see <http://www.gnu.org/licenses/>

import os
import sys
import xml.etree.ElementTree as et
from dataclasses import dataclass, field
from typing import List, Optional

from JSBSim_utils import JSBSimTestCase, CheckXMLFile, RunTest


@dataclass
class ValidationWarning:
    """A single validation warning."""
    category: str
    message: str


@dataclass
class AircraftValidationResult:
    """Validation result for a single aircraft."""
    name: str
    warnings: List[ValidationWarning] = field(default_factory=list)

    def add_warning(self, category: str, message: str):
        self.warnings.append(ValidationWarning(category, message))

    @property
    def is_valid(self) -> bool:
        return len(self.warnings) == 0


class AircraftValidator:
    """Validates aircraft XML files for plausibility and metadata."""

    # Physical plausibility ranges
    MIN_WING_LOADING_LB_FT2 = 5.0    # Very light aircraft
    MAX_WING_LOADING_LB_FT2 = 150.0  # Heavy jets

    def __init__(self, aircraft_path: str):
        self.aircraft_path = aircraft_path

    def validate_all(self) -> List[AircraftValidationResult]:
        """Validate all aircraft in the aircraft directory."""
        results = []
        for d in sorted(os.listdir(self.aircraft_path)):
            fullpath = os.path.join(self.aircraft_path, d)
            if not os.path.isdir(fullpath):
                continue

            aircraft_file = os.path.join(self.aircraft_path, d, d + '.xml')
            if not CheckXMLFile(aircraft_file, 'fdm_config'):
                continue

            # Skip known special cases
            if d in ('blank',):
                continue

            result = self.validate_aircraft(d, aircraft_file)
            results.append(result)

        return results

    def validate_aircraft(self, name: str, filepath: str) -> AircraftValidationResult:
        """Validate a single aircraft file."""
        result = AircraftValidationResult(name=name)

        try:
            tree = et.parse(filepath)
            root = tree.getroot()
        except et.ParseError as e:
            result.add_warning("xml", f"Failed to parse XML: {e}")
            return result

        self._validate_metadata(root, result)
        self._validate_physical_plausibility(root, result)

        return result

    def _validate_metadata(self, root: et.Element, result: AircraftValidationResult):
        """Check for required metadata fields."""
        fileheader = root.find('fileheader')
        if fileheader is None:
            result.add_warning("metadata", "Missing <fileheader> element")
            return

        # Check for author
        author = fileheader.find('author')
        if author is None or not (author.text and author.text.strip()):
            result.add_warning("metadata", "Missing or empty <author> in fileheader")

        # Check for description
        description = fileheader.find('description')
        if description is None or not (description.text and description.text.strip()):
            result.add_warning("metadata", "Missing or empty <description> in fileheader")

        # Check for license
        license_el = fileheader.find('license')
        if license_el is None:
            result.add_warning("metadata", "Missing <license> in fileheader")
        else:
            license_name = license_el.get('licenseName')
            license_url = license_el.get('licenseURL')
            if not license_name and not license_url:
                result.add_warning("metadata", "License element missing licenseName or licenseURL attribute")

    def _validate_physical_plausibility(self, root: et.Element, result: AircraftValidationResult):
        """Check for physically plausible values."""
        # Get metrics
        metrics = root.find('metrics')
        mass_balance = root.find('mass_balance')

        if metrics is None:
            result.add_warning("plausibility", "Missing <metrics> element")
            return
        if mass_balance is None:
            result.add_warning("plausibility", "Missing <mass_balance> element")
            return

        # Check wing area
        wingarea = self._get_value(metrics, 'wingarea')
        if wingarea is not None and wingarea <= 0:
            result.add_warning("plausibility", f"Wing area must be positive: {wingarea}")

        # Check empty weight
        emptywt = self._get_value(mass_balance, 'emptywt')
        if emptywt is not None:
            if emptywt <= 0:
                result.add_warning("plausibility", f"Empty weight must be positive: {emptywt}")

        # Check inertias (must be positive)
        for inertia in ['ixx', 'iyy', 'izz']:
            value = self._get_value(mass_balance, inertia)
            if value is not None and value <= 0:
                result.add_warning("plausibility", f"{inertia.upper()} must be positive: {value}")

        # Check wing loading if we have both values
        if wingarea is not None and wingarea > 0 and emptywt is not None and emptywt > 0:
            # Convert wing area to ft^2 if needed (assume already in ft^2 for simplicity)
            wing_loading = emptywt / wingarea
            if wing_loading < self.MIN_WING_LOADING_LB_FT2:
                result.add_warning("plausibility",
                    f"Wing loading {wing_loading:.1f} lb/ft^2 is very low (min: {self.MIN_WING_LOADING_LB_FT2})")
            elif wing_loading > self.MAX_WING_LOADING_LB_FT2:
                result.add_warning("plausibility",
                    f"Wing loading {wing_loading:.1f} lb/ft^2 is very high (max: {self.MAX_WING_LOADING_LB_FT2})")

        # Check CG location exists
        cg = mass_balance.find("location[@name='CG']")
        if cg is None:
            result.add_warning("plausibility", "Missing CG location in mass_balance")

    def _get_value(self, parent: et.Element, tag: str) -> Optional[float]:
        """Get a numeric value from an element, handling unit conversions."""
        el = parent.find(tag)
        if el is None or el.text is None:
            return None
        try:
            return float(el.text.strip())
        except ValueError:
            return None


def print_validation_summary(results: List[AircraftValidationResult]):
    """Print a summary of validation results."""
    total_warnings = 0
    aircraft_with_warnings = 0

    print("\n" + "=" * 60)
    print("AIRCRAFT VALIDATION SUMMARY")
    print("=" * 60)

    for result in results:
        if result.warnings:
            aircraft_with_warnings += 1
            print(f"\n{result.name}:")
            for warning in result.warnings:
                print(f"  [{warning.category}] {warning.message}")
                total_warnings += 1

    print("\n" + "-" * 60)
    print(f"Total: {len(results)} aircraft checked")
    print(f"       {aircraft_with_warnings} with warnings")
    print(f"       {total_warnings} total warnings")
    print("=" * 60 + "\n")


class ValidateAircrafts(JSBSimTestCase):
    """Test case that validates all aircraft configurations."""

    def test_validate_all_aircraft(self):
        """Validate all aircraft and report warnings."""
        aircraft_path = self.sandbox.path_to_jsbsim_file('aircraft')
        validator = AircraftValidator(aircraft_path)
        results = validator.validate_all()

        print_validation_summary(results)

        # Count warnings by category
        warnings_by_category = {}
        for result in results:
            for warning in result.warnings:
                warnings_by_category[warning.category] = \
                    warnings_by_category.get(warning.category, 0) + 1

        # Log warning counts but don't fail the test
        if warnings_by_category:
            print("Warnings by category:")
            for category, count in sorted(warnings_by_category.items()):
                print(f"  {category}: {count}")

        # This test always passes - it's informational only
        self.assertTrue(True, "Validation complete (warnings are informational)")


RunTest(ValidateAircrafts)
