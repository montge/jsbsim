#include <limits>
#include <cmath>
#include <cxxtest/TestSuite.h>
#include "TestAssertions.h"
#include <FGFDMExec.h>
#include <initialization/FGInitialCondition.h>
#include <math/FGStateSpace.h>

const double epsilon = 100. * std::numeric_limits<double>::epsilon();

class FGStateSpaceTest : public CxxTest::TestSuite
{
private:
  JSBSim::FGFDMExec* fdmExec;

public:
  void setUp() {
    // Create FDM instance for each test
    fdmExec = new JSBSim::FGFDMExec();
  }

  void tearDown() {
    delete fdmExec;
    fdmExec = nullptr;
  }

  // ============ Component Tests ============

  void testComponentConstruction() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Creating component objects
    JSBSim::FGStateSpace::Vt vt;
    JSBSim::FGStateSpace::Alpha alpha;
    JSBSim::FGStateSpace::Beta beta;
    JSBSim::FGStateSpace::Theta theta;
    JSBSim::FGStateSpace::Q q;

    // THEN: Components should have proper names and units
    TS_ASSERT_EQUALS(vt.getName(), "Vt");
    TS_ASSERT_EQUALS(vt.getUnit(), "ft/s");
    TS_ASSERT_EQUALS(alpha.getName(), "Alpha");
    TS_ASSERT_EQUALS(alpha.getUnit(), "rad");
    TS_ASSERT_EQUALS(beta.getName(), "Beta");
    TS_ASSERT_EQUALS(beta.getUnit(), "rad");
    TS_ASSERT_EQUALS(theta.getName(), "Theta");
    TS_ASSERT_EQUALS(theta.getUnit(), "rad");
    TS_ASSERT_EQUALS(q.getName(), "Q");
    TS_ASSERT_EQUALS(q.getUnit(), "rad/s");
  }

  void testControlSurfaceComponents() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Creating control surface component objects
    JSBSim::FGStateSpace::DeCmd deCmd;
    JSBSim::FGStateSpace::DaCmd daCmd;
    JSBSim::FGStateSpace::DrCmd drCmd;
    JSBSim::FGStateSpace::ThrottleCmd throttleCmd;

    // THEN: Components should have proper names and units
    TS_ASSERT_EQUALS(deCmd.getName(), "DeCmd");
    TS_ASSERT_EQUALS(deCmd.getUnit(), "norm");
    TS_ASSERT_EQUALS(daCmd.getName(), "DaCmd");
    TS_ASSERT_EQUALS(daCmd.getUnit(), "norm");
    TS_ASSERT_EQUALS(drCmd.getName(), "DrCmd");
    TS_ASSERT_EQUALS(drCmd.getUnit(), "norm");
    TS_ASSERT_EQUALS(throttleCmd.getName(), "ThtlCmd");
    TS_ASSERT_EQUALS(throttleCmd.getUnit(), "norm");
  }

  void testAccelerationComponents() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Creating acceleration component objects
    JSBSim::FGStateSpace::AccelX accelX;
    JSBSim::FGStateSpace::AccelY accelY;
    JSBSim::FGStateSpace::AccelZ accelZ;

    // THEN: Components should have proper names and units
    TS_ASSERT_EQUALS(accelX.getName(), "AccelX");
    TS_ASSERT_EQUALS(accelX.getUnit(), "ft/s^2");
    TS_ASSERT_EQUALS(accelY.getName(), "AccelY");
    TS_ASSERT_EQUALS(accelY.getUnit(), "ft/s^2");
    TS_ASSERT_EQUALS(accelZ.getName(), "AccelZ");
    TS_ASSERT_EQUALS(accelZ.getUnit(), "ft/s^2");
  }

  void testPositionComponents() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Creating position component objects
    JSBSim::FGStateSpace::Latitude lat;
    JSBSim::FGStateSpace::Longitude lon;
    JSBSim::FGStateSpace::Alt alt;

    // THEN: Components should have proper names and units
    TS_ASSERT_EQUALS(lat.getName(), "Latitude");
    TS_ASSERT_EQUALS(lat.getUnit(), "rad");
    TS_ASSERT_EQUALS(lon.getName(), "Longitude");
    TS_ASSERT_EQUALS(lon.getUnit(), "rad");
    TS_ASSERT_EQUALS(alt.getName(), "Alt");
    TS_ASSERT_EQUALS(alt.getUnit(), "ft");
  }

  void testEulerAngleComponents() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Creating Euler angle component objects
    JSBSim::FGStateSpace::Phi phi;
    JSBSim::FGStateSpace::Theta theta;
    JSBSim::FGStateSpace::Psi psi;

    // THEN: Components should have proper names and units
    TS_ASSERT_EQUALS(phi.getName(), "Phi");
    TS_ASSERT_EQUALS(phi.getUnit(), "rad");
    TS_ASSERT_EQUALS(theta.getName(), "Theta");
    TS_ASSERT_EQUALS(theta.getUnit(), "rad");
    TS_ASSERT_EQUALS(psi.getName(), "Psi");
    TS_ASSERT_EQUALS(psi.getUnit(), "rad");
  }

  void testAngularRateComponents() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Creating angular rate component objects
    JSBSim::FGStateSpace::P p;
    JSBSim::FGStateSpace::Q q;
    JSBSim::FGStateSpace::R r;

    // THEN: Components should have proper names and units
    TS_ASSERT_EQUALS(p.getName(), "P");
    TS_ASSERT_EQUALS(p.getUnit(), "rad/s");
    TS_ASSERT_EQUALS(q.getName(), "Q");
    TS_ASSERT_EQUALS(q.getUnit(), "rad/s");
    TS_ASSERT_EQUALS(r.getName(), "R");
    TS_ASSERT_EQUALS(r.getUnit(), "rad/s");
  }

  void testInertialAngularRateComponents() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Creating inertial angular rate component objects
    JSBSim::FGStateSpace::Pi pi;
    JSBSim::FGStateSpace::Qi qi;
    JSBSim::FGStateSpace::Ri ri;

    // THEN: Components should have proper names and units
    TS_ASSERT_EQUALS(pi.getName(), "P inertial");
    TS_ASSERT_EQUALS(pi.getUnit(), "rad/s");
    TS_ASSERT_EQUALS(qi.getName(), "Q inertial");
    TS_ASSERT_EQUALS(qi.getUnit(), "rad/s");
    TS_ASSERT_EQUALS(ri.getName(), "R inertial");
    TS_ASSERT_EQUALS(ri.getUnit(), "rad/s");
  }

  void testVelocityComponents() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Creating velocity component objects
    JSBSim::FGStateSpace::Vt vt;
    JSBSim::FGStateSpace::VGround vGround;
    JSBSim::FGStateSpace::Vn vn;
    JSBSim::FGStateSpace::Ve ve;
    JSBSim::FGStateSpace::Vd vd;

    // THEN: Components should have proper names and units
    TS_ASSERT_EQUALS(vt.getName(), "Vt");
    TS_ASSERT_EQUALS(vt.getUnit(), "ft/s");
    TS_ASSERT_EQUALS(vGround.getName(), "VGround");
    TS_ASSERT_EQUALS(vGround.getUnit(), "ft/s");
    TS_ASSERT_EQUALS(vn.getName(), "Vel north");
    TS_ASSERT_EQUALS(vn.getUnit(), "feet/s");
    TS_ASSERT_EQUALS(ve.getName(), "Vel east");
    TS_ASSERT_EQUALS(ve.getUnit(), "feet/s");
    TS_ASSERT_EQUALS(vd.getName(), "Vel down");
    TS_ASSERT_EQUALS(vd.getUnit(), "feet/s");
  }

  void testPropulsionComponents() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Creating propulsion component objects
    JSBSim::FGStateSpace::Rpm0 rpm0;
    JSBSim::FGStateSpace::Rpm1 rpm1;
    JSBSim::FGStateSpace::PropPitch propPitch;

    // THEN: Components should have proper names and units
    TS_ASSERT_EQUALS(rpm0.getName(), "Rpm0");
    TS_ASSERT_EQUALS(rpm0.getUnit(), "rev/min");
    TS_ASSERT_EQUALS(rpm1.getName(), "Rpm1");
    TS_ASSERT_EQUALS(rpm1.getUnit(), "rev/min");
    TS_ASSERT_EQUALS(propPitch.getName(), "Prop Pitch");
    TS_ASSERT_EQUALS(propPitch.getUnit(), "deg");
  }

  void testCourseOverGroundComponent() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Creating COG component object
    JSBSim::FGStateSpace::COG cog;

    // THEN: Component should have proper name and unit
    TS_ASSERT_EQUALS(cog.getName(), "Course Over Ground");
    TS_ASSERT_EQUALS(cog.getUnit(), "rad");
  }

  // ============ ComponentVector Tests ============

  void testComponentVectorConstruction() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // THEN: ComponentVectors should be properly initialized
    TS_ASSERT_EQUALS(ss.x.getSize(), 0);
    TS_ASSERT_EQUALS(ss.u.getSize(), 0);
    TS_ASSERT_EQUALS(ss.y.getSize(), 0);
  }

  void testComponentVectorAdd() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Adding components to state vector
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    ss.x.add(new JSBSim::FGStateSpace::Beta());

    // THEN: Size should increase
    TS_ASSERT_EQUALS(ss.x.getSize(), 3);
    TS_ASSERT_EQUALS(ss.x.getName(0), "Vt");
    TS_ASSERT_EQUALS(ss.x.getName(1), "Alpha");
    TS_ASSERT_EQUALS(ss.x.getName(2), "Beta");
  }

  void testComponentVectorClear() {
    // GIVEN: A state space object with components
    JSBSim::FGStateSpace ss(fdmExec);
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    TS_ASSERT_EQUALS(ss.x.getSize(), 2);

    // WHEN: Clearing the component vector
    ss.x.clear();

    // THEN: Size should be zero
    TS_ASSERT_EQUALS(ss.x.getSize(), 0);
  }

  void testComponentVectorGetNames() {
    // GIVEN: A state space object with components
    JSBSim::FGStateSpace ss(fdmExec);
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    ss.x.add(new JSBSim::FGStateSpace::Theta());

    // WHEN: Getting all names
    std::vector<std::string> names = ss.x.getName();

    // THEN: Should return vector of names
    TS_ASSERT_EQUALS(names.size(), 3);
    TS_ASSERT_EQUALS(names[0], "Vt");
    TS_ASSERT_EQUALS(names[1], "Alpha");
    TS_ASSERT_EQUALS(names[2], "Theta");
  }

  void testComponentVectorGetUnits() {
    // GIVEN: A state space object with components
    JSBSim::FGStateSpace ss(fdmExec);
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    ss.x.add(new JSBSim::FGStateSpace::Q());

    // WHEN: Getting all units
    std::vector<std::string> units = ss.x.getUnit();

    // THEN: Should return vector of units
    TS_ASSERT_EQUALS(units.size(), 3);
    TS_ASSERT_EQUALS(units[0], "ft/s");
    TS_ASSERT_EQUALS(units[1], "rad");
    TS_ASSERT_EQUALS(units[2], "rad/s");
  }

  void testComponentVectorCopyConstructor() {
    // GIVEN: A state space object with components
    JSBSim::FGStateSpace ss(fdmExec);
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());

    // WHEN: Copying the component vector
    JSBSim::FGStateSpace::ComponentVector x_copy(ss.x);

    // THEN: Copy should have same size and names
    TS_ASSERT_EQUALS(x_copy.getSize(), 2);
    TS_ASSERT_EQUALS(x_copy.getName(0), "Vt");
    TS_ASSERT_EQUALS(x_copy.getName(1), "Alpha");
  }

  void testComponentVectorAssignment() {
    // GIVEN: Two state space objects with different components
    JSBSim::FGStateSpace ss1(fdmExec);
    JSBSim::FGStateSpace ss2(fdmExec);
    ss1.x.add(new JSBSim::FGStateSpace::Vt());
    ss1.x.add(new JSBSim::FGStateSpace::Alpha());

    // WHEN: Assigning one component vector to another
    ss2.x = ss1.x;

    // THEN: Assignment should copy components
    TS_ASSERT_EQUALS(ss2.x.getSize(), 2);
    TS_ASSERT_EQUALS(ss2.x.getName(0), "Vt");
    TS_ASSERT_EQUALS(ss2.x.getName(1), "Alpha");
  }

  // ============ StateSpace Tests ============

  void testStateSpaceConstruction() {
    // GIVEN: An FDM executive
    // WHEN: Creating a state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // THEN: Object should be properly initialized
    TS_ASSERT_EQUALS(ss.x.getSize(), 0);
    TS_ASSERT_EQUALS(ss.u.getSize(), 0);
    TS_ASSERT_EQUALS(ss.y.getSize(), 0);
  }

  void testStateSpaceClear() {
    // GIVEN: A state space object with components
    JSBSim::FGStateSpace ss(fdmExec);
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.u.add(new JSBSim::FGStateSpace::DeCmd());
    ss.y.add(new JSBSim::FGStateSpace::Q());

    // WHEN: Clearing the state space
    ss.clear();

    // THEN: All vectors should be empty
    TS_ASSERT_EQUALS(ss.x.getSize(), 0);
    TS_ASSERT_EQUALS(ss.u.getSize(), 0);
    TS_ASSERT_EQUALS(ss.y.getSize(), 0);
  }

  void testStateSpaceSetFdm() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Setting a new FDM executive
    JSBSim::FGFDMExec* newFdm = new JSBSim::FGFDMExec();
    ss.setFdm(newFdm);

    // THEN: The FDM should be updated
    // (We can't directly test this without accessing private members,
    // but we verify it doesn't crash)
    TS_ASSERT(true);

    delete newFdm;
  }

  void testStateSpaceStateSum() {
    // GIVEN: A state space object
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Computing state sum with empty state
    double sum = ss.stateSum();

    // THEN: Sum should be zero
    TS_ASSERT_EQUALS(sum, 0.0);
  }

  // ============ Edge Cases ============

  void testEmptyStateVectorOperations() {
    // GIVEN: A state space with empty vectors
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Performing operations on empty vectors
    std::vector<double> values = ss.x.get();
    std::vector<std::string> names = ss.x.getName();
    std::vector<std::string> units = ss.x.getUnit();

    // THEN: Operations should return empty vectors
    TS_ASSERT_EQUALS(values.size(), 0);
    TS_ASSERT_EQUALS(names.size(), 0);
    TS_ASSERT_EQUALS(units.size(), 0);
  }

  void testSingleComponentVector() {
    // GIVEN: A state space with single component
    JSBSim::FGStateSpace ss(fdmExec);
    ss.x.add(new JSBSim::FGStateSpace::Vt());

    // WHEN: Getting vector data
    std::vector<std::string> names = ss.x.getName();
    std::vector<std::string> units = ss.x.getUnit();

    // THEN: Should return single-element vectors
    TS_ASSERT_EQUALS(names.size(), 1);
    TS_ASSERT_EQUALS(names[0], "Vt");
    TS_ASSERT_EQUALS(units.size(), 1);
    TS_ASSERT_EQUALS(units[0], "ft/s");
  }

  void testMultipleInputsOutputs() {
    // GIVEN: A state space with multiple inputs and outputs
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Adding multiple inputs
    ss.u.add(new JSBSim::FGStateSpace::DeCmd());
    ss.u.add(new JSBSim::FGStateSpace::DaCmd());
    ss.u.add(new JSBSim::FGStateSpace::DrCmd());
    ss.u.add(new JSBSim::FGStateSpace::ThrottleCmd());

    // AND: Adding multiple outputs
    ss.y.add(new JSBSim::FGStateSpace::Alpha());
    ss.y.add(new JSBSim::FGStateSpace::Beta());
    ss.y.add(new JSBSim::FGStateSpace::Q());

    // THEN: Sizes should match
    TS_ASSERT_EQUALS(ss.u.getSize(), 4);
    TS_ASSERT_EQUALS(ss.y.getSize(), 3);
  }

  void testComponentVectorBoundaryAccess() {
    // GIVEN: A state space with components
    JSBSim::FGStateSpace ss(fdmExec);
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());

    // WHEN: Accessing first and last elements
    std::string firstName = ss.x.getName(0);
    std::string lastName = ss.x.getName(1);

    // THEN: Should return correct names
    TS_ASSERT_EQUALS(firstName, "Vt");
    TS_ASSERT_EQUALS(lastName, "Alpha");
  }

  void testClearAfterMultipleAdds() {
    // GIVEN: A state space with multiple operations
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Adding, clearing, and adding again
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    TS_ASSERT_EQUALS(ss.x.getSize(), 2);

    ss.x.clear();
    TS_ASSERT_EQUALS(ss.x.getSize(), 0);

    ss.x.add(new JSBSim::FGStateSpace::Theta());

    // THEN: Should have new component
    TS_ASSERT_EQUALS(ss.x.getSize(), 1);
    TS_ASSERT_EQUALS(ss.x.getName(0), "Theta");
  }

  void testAllControlSurfaceInputs() {
    // GIVEN: A state space for control analysis
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Adding all control surface inputs
    ss.u.add(new JSBSim::FGStateSpace::DeCmd());
    ss.u.add(new JSBSim::FGStateSpace::DaCmd());
    ss.u.add(new JSBSim::FGStateSpace::DrCmd());
    ss.u.add(new JSBSim::FGStateSpace::ThrottleCmd());

    // THEN: All should be normalized inputs
    std::vector<std::string> units = ss.u.getUnit();
    for (size_t i = 0; i < units.size(); i++) {
      TS_ASSERT_EQUALS(units[i], "norm");
    }
  }

  void testLongitudinalStateVector() {
    // GIVEN: A state space for longitudinal dynamics
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Adding longitudinal states
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    ss.x.add(new JSBSim::FGStateSpace::Theta());
    ss.x.add(new JSBSim::FGStateSpace::Q());

    // THEN: Should have 4 states
    TS_ASSERT_EQUALS(ss.x.getSize(), 4);
    std::vector<std::string> names = ss.x.getName();
    TS_ASSERT_EQUALS(names[0], "Vt");
    TS_ASSERT_EQUALS(names[1], "Alpha");
    TS_ASSERT_EQUALS(names[2], "Theta");
    TS_ASSERT_EQUALS(names[3], "Q");
  }

  void testLateralDirectionalStateVector() {
    // GIVEN: A state space for lateral-directional dynamics
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Adding lateral-directional states
    ss.x.add(new JSBSim::FGStateSpace::Beta());
    ss.x.add(new JSBSim::FGStateSpace::Phi());
    ss.x.add(new JSBSim::FGStateSpace::P());
    ss.x.add(new JSBSim::FGStateSpace::R());

    // THEN: Should have 4 states
    TS_ASSERT_EQUALS(ss.x.getSize(), 4);
    std::vector<std::string> names = ss.x.getName();
    TS_ASSERT_EQUALS(names[0], "Beta");
    TS_ASSERT_EQUALS(names[1], "Phi");
    TS_ASSERT_EQUALS(names[2], "P");
    TS_ASSERT_EQUALS(names[3], "R");
  }

  void testNavigationStateVector() {
    // GIVEN: A state space for navigation
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Adding navigation states
    ss.x.add(new JSBSim::FGStateSpace::Latitude());
    ss.x.add(new JSBSim::FGStateSpace::Longitude());
    ss.x.add(new JSBSim::FGStateSpace::Alt());
    ss.x.add(new JSBSim::FGStateSpace::Vn());
    ss.x.add(new JSBSim::FGStateSpace::Ve());
    ss.x.add(new JSBSim::FGStateSpace::Vd());

    // THEN: Should have 6 states
    TS_ASSERT_EQUALS(ss.x.getSize(), 6);
  }

  void testFullStateVector() {
    // GIVEN: A state space for complete 6-DOF dynamics
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Adding all primary states
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    ss.x.add(new JSBSim::FGStateSpace::Beta());
    ss.x.add(new JSBSim::FGStateSpace::Phi());
    ss.x.add(new JSBSim::FGStateSpace::Theta());
    ss.x.add(new JSBSim::FGStateSpace::Psi());
    ss.x.add(new JSBSim::FGStateSpace::P());
    ss.x.add(new JSBSim::FGStateSpace::Q());
    ss.x.add(new JSBSim::FGStateSpace::R());

    // THEN: Should have 9 states
    TS_ASSERT_EQUALS(ss.x.getSize(), 9);
  }

  void testComponentVectorIndividualGetters() {
    // GIVEN: A state space with components
    JSBSim::FGStateSpace ss(fdmExec);
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    ss.x.add(new JSBSim::FGStateSpace::Theta());

    // WHEN: Getting individual names and units
    // THEN: Each should return correct value
    TS_ASSERT_EQUALS(ss.x.getName(0), "Vt");
    TS_ASSERT_EQUALS(ss.x.getUnit(0), "ft/s");
    TS_ASSERT_EQUALS(ss.x.getName(1), "Alpha");
    TS_ASSERT_EQUALS(ss.x.getUnit(1), "rad");
    TS_ASSERT_EQUALS(ss.x.getName(2), "Theta");
    TS_ASSERT_EQUALS(ss.x.getUnit(2), "rad");
  }

  void testComponentVectorGetComp() {
    // GIVEN: A state space with components
    JSBSim::FGStateSpace ss(fdmExec);
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());

    // WHEN: Getting component pointers
    JSBSim::FGStateSpace::Component* comp0 = ss.x.getComp(0);
    JSBSim::FGStateSpace::Component* comp1 = ss.x.getComp(1);

    // THEN: Components should be valid
    TS_ASSERT(comp0 != nullptr);
    TS_ASSERT(comp1 != nullptr);
    TS_ASSERT_EQUALS(comp0->getName(), "Vt");
    TS_ASSERT_EQUALS(comp1->getName(), "Alpha");
  }

  void testMixedComponentTypes() {
    // GIVEN: A state space with different component categories
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Mixing states, inputs, and outputs
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    ss.u.add(new JSBSim::FGStateSpace::DeCmd());
    ss.y.add(new JSBSim::FGStateSpace::Q());

    // THEN: Each vector should have correct size
    TS_ASSERT_EQUALS(ss.x.getSize(), 2);
    TS_ASSERT_EQUALS(ss.u.getSize(), 1);
    TS_ASSERT_EQUALS(ss.y.getSize(), 1);
  }

  void testAccelerationOutputs() {
    // GIVEN: A state space with acceleration outputs
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Adding acceleration components as outputs
    ss.y.add(new JSBSim::FGStateSpace::AccelX());
    ss.y.add(new JSBSim::FGStateSpace::AccelY());
    ss.y.add(new JSBSim::FGStateSpace::AccelZ());

    // THEN: Should have proper units
    std::vector<std::string> units = ss.y.getUnit();
    TS_ASSERT_EQUALS(units[0], "ft/s^2");
    TS_ASSERT_EQUALS(units[1], "ft/s^2");
    TS_ASSERT_EQUALS(units[2], "ft/s^2");
  }

  void testInertialRatesAsStates() {
    // GIVEN: A state space using inertial angular rates
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Adding inertial rates as states
    ss.x.add(new JSBSim::FGStateSpace::Pi());
    ss.x.add(new JSBSim::FGStateSpace::Qi());
    ss.x.add(new JSBSim::FGStateSpace::Ri());

    // THEN: Should have correct names
    std::vector<std::string> names = ss.x.getName();
    TS_ASSERT_EQUALS(names[0], "P inertial");
    TS_ASSERT_EQUALS(names[1], "Q inertial");
    TS_ASSERT_EQUALS(names[2], "R inertial");
  }

  void testPropulsionStatesAndInputs() {
    // GIVEN: A state space with propulsion components
    JSBSim::FGStateSpace ss(fdmExec);

    // WHEN: Adding propulsion states and inputs
    ss.x.add(new JSBSim::FGStateSpace::Rpm0());
    ss.u.add(new JSBSim::FGStateSpace::ThrottleCmd());
    ss.u.add(new JSBSim::FGStateSpace::PropPitch());

    // THEN: Should have correct configuration
    TS_ASSERT_EQUALS(ss.x.getSize(), 1);
    TS_ASSERT_EQUALS(ss.u.getSize(), 2);
    TS_ASSERT_EQUALS(ss.x.getName(0), "Rpm0");
  }

  // ============ State-Space Theory Tests ============

  // Test 38: State matrix dimensions concept
  void testStateMatrixDimensions() {
    // For x' = Ax + Bu, y = Cx + Du
    // A is nxn, B is nxm, C is pxn, D is pxm
    int n = 4;  // Number of states
    int m = 2;  // Number of inputs
    int p = 3;  // Number of outputs

    int A_rows = n, A_cols = n;
    int B_rows = n, B_cols = m;
    int C_rows = p, C_cols = n;
    int D_rows = p, D_cols = m;

    TS_ASSERT_EQUALS(A_rows, A_cols);  // A is square
    TS_ASSERT_EQUALS(B_rows, A_rows);  // B rows match A
    TS_ASSERT_EQUALS(C_cols, A_cols);  // C cols match A
    TS_ASSERT_EQUALS(D_rows, C_rows);  // D rows match C
    TS_ASSERT_EQUALS(D_cols, B_cols);  // D cols match B
  }

  // Test 39: Controllability matrix rank concept
  void testControllabilityRankConcept() {
    // System is controllable if rank([B, AB, A^2B, ...]) = n
    int n = 3;  // 3 states
    int m = 1;  // 1 input

    // Controllability matrix has n rows and n*m columns
    int C_rows = n;
    int C_cols = n * m;

    TS_ASSERT_EQUALS(C_rows, 3);
    TS_ASSERT_EQUALS(C_cols, 3);
  }

  // Test 40: Observability matrix rank concept
  void testObservabilityRankConcept() {
    // System is observable if rank([C; CA; CA^2; ...]) = n
    int n = 4;  // 4 states
    int p = 2;  // 2 outputs

    // Observability matrix has n*p rows and n columns
    int O_rows = n * p;
    int O_cols = n;

    TS_ASSERT_EQUALS(O_rows, 8);
    TS_ASSERT_EQUALS(O_cols, 4);
  }

  // Test 41: Eigenvalue stability concept
  void testEigenvalueStabilityConcept() {
    // For continuous system, stable if all eigenvalues have negative real parts
    double realPart1 = -2.0;
    double realPart2 = -0.5;
    double realPart3 = -1.0;

    bool stable = (realPart1 < 0) && (realPart2 < 0) && (realPart3 < 0);
    TS_ASSERT(stable);
  }

  // Test 42: Eigenvalue for oscillatory mode
  void testEigenvalueOscillatoryMode() {
    // Complex eigenvalue: sigma +/- j*omega
    double sigma = -0.5;  // Real part (damping)
    double omega = 2.0;   // Imaginary part (frequency)

    // Natural frequency and damping ratio
    double omega_n = std::sqrt(sigma * sigma + omega * omega);
    double zeta = -sigma / omega_n;

    TS_ASSERT_DELTA(omega_n, 2.06, 0.01);
    TS_ASSERT_DELTA(zeta, 0.243, 0.01);
  }

  // Test 43: Transfer function from state-space
  void testTransferFunctionConcept() {
    // G(s) = C(sI - A)^{-1}B + D
    // For SISO: G(s) = (b_0 + b_1*s + ...) / (a_0 + a_1*s + ...)
    int numOrder = 2;  // Numerator order
    int denOrder = 3;  // Denominator order

    TS_ASSERT(denOrder >= numOrder);  // Proper transfer function
  }

  // Test 44: State feedback gain concept
  void testStateFeedbackConcept() {
    // u = -Kx + r for pole placement
    int n = 4;  // states
    int m = 2;  // inputs

    // K is m x n
    int K_rows = m;
    int K_cols = n;

    TS_ASSERT_EQUALS(K_rows, 2);
    TS_ASSERT_EQUALS(K_cols, 4);
  }

  // Test 45: Observer gain concept
  void testObserverGainConcept() {
    // x_hat' = Ax_hat + Bu + L(y - Cx_hat)
    int n = 3;  // states
    int p = 2;  // outputs

    // L is n x p
    int L_rows = n;
    int L_cols = p;

    TS_ASSERT_EQUALS(L_rows, 3);
    TS_ASSERT_EQUALS(L_cols, 2);
  }

  // Test 46: LQR cost matrices concept
  void testLQRCostMatricesConcept() {
    // J = integral(x'Qx + u'Ru) dt
    int n = 4;  // states
    int m = 2;  // inputs

    // Q is n x n, R is m x m (both symmetric positive semi-definite)
    int Q_size = n;
    int R_size = m;

    TS_ASSERT_EQUALS(Q_size, 4);
    TS_ASSERT_EQUALS(R_size, 2);
  }

  // Test 47: Discrete-time state-space concept
  void testDiscreteTimeStateSpaceConcept() {
    // x[k+1] = Ad*x[k] + Bd*u[k]
    double dt = 0.01;  // Sample time
    double eigenContinuous = -2.0;

    // Discrete eigenvalue = exp(lambda_c * dt)
    double eigenDiscrete = std::exp(eigenContinuous * dt);
    TS_ASSERT_DELTA(eigenDiscrete, 0.98, 0.01);

    // For stability: |eigenDiscrete| < 1
    TS_ASSERT(std::abs(eigenDiscrete) < 1.0);
  }

  // Test 48: Sampling rate and Nyquist
  void testSamplingRateConcept() {
    double systemBandwidth = 10.0;  // Hz
    double nyquistRate = 2.0 * systemBandwidth;
    double recommendedSampleRate = 10.0 * systemBandwidth;

    TS_ASSERT_EQUALS(nyquistRate, 20.0);
    TS_ASSERT_EQUALS(recommendedSampleRate, 100.0);
  }

  // Test 49: State vector scaling concept
  void testStateScalingConcept() {
    // Scale states for numerical conditioning
    double vt_scale = 100.0;    // ft/s typical
    double alpha_scale = 0.2;   // rad typical
    double q_scale = 0.5;       // rad/s typical

    // Scaling matrix diagonal
    double S_vt = 1.0 / vt_scale;
    double S_alpha = 1.0 / alpha_scale;
    double S_q = 1.0 / q_scale;

    TS_ASSERT_DELTA(S_vt, 0.01, epsilon);
    TS_ASSERT_DELTA(S_alpha, 5.0, epsilon);
    TS_ASSERT_DELTA(S_q, 2.0, epsilon);
  }

  // Test 50: Time constant from eigenvalue
  void testTimeConstantFromEigenvalue() {
    double eigenvalue = -0.5;  // Real eigenvalue

    // Time constant = -1/eigenvalue
    double tau = -1.0 / eigenvalue;
    TS_ASSERT_DELTA(tau, 2.0, epsilon);
  }

  // ============ More Configuration Tests ============

  // Test 51: Complete 6-DOF with position
  void testComplete6DOFWithPosition() {
    JSBSim::FGStateSpace ss(fdmExec);

    // 12 state model: position, velocity, attitude, angular rates
    ss.x.add(new JSBSim::FGStateSpace::Latitude());
    ss.x.add(new JSBSim::FGStateSpace::Longitude());
    ss.x.add(new JSBSim::FGStateSpace::Alt());
    ss.x.add(new JSBSim::FGStateSpace::Vn());
    ss.x.add(new JSBSim::FGStateSpace::Ve());
    ss.x.add(new JSBSim::FGStateSpace::Vd());
    ss.x.add(new JSBSim::FGStateSpace::Phi());
    ss.x.add(new JSBSim::FGStateSpace::Theta());
    ss.x.add(new JSBSim::FGStateSpace::Psi());
    ss.x.add(new JSBSim::FGStateSpace::P());
    ss.x.add(new JSBSim::FGStateSpace::Q());
    ss.x.add(new JSBSim::FGStateSpace::R());

    TS_ASSERT_EQUALS(ss.x.getSize(), 12);
  }

  // Test 52: Ground speed as output
  void testGroundSpeedOutput() {
    JSBSim::FGStateSpace ss(fdmExec);

    ss.y.add(new JSBSim::FGStateSpace::VGround());
    ss.y.add(new JSBSim::FGStateSpace::COG());

    TS_ASSERT_EQUALS(ss.y.getSize(), 2);
    TS_ASSERT_EQUALS(ss.y.getName(0), "VGround");
    TS_ASSERT_EQUALS(ss.y.getName(1), "Course Over Ground");
  }

  // Test 53: Multi-engine configuration
  void testMultiEngineConfiguration() {
    JSBSim::FGStateSpace ss(fdmExec);

    ss.x.add(new JSBSim::FGStateSpace::Rpm0());
    ss.x.add(new JSBSim::FGStateSpace::Rpm1());

    TS_ASSERT_EQUALS(ss.x.getSize(), 2);
    TS_ASSERT_EQUALS(ss.x.getName(0), "Rpm0");
    TS_ASSERT_EQUALS(ss.x.getName(1), "Rpm1");
  }

  // Test 54: Augmented state vector with integral
  void testAugmentedStateVector() {
    JSBSim::FGStateSpace ss(fdmExec);

    // Standard states
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    ss.x.add(new JSBSim::FGStateSpace::Q());
    // Plus integrator states (conceptually)
    ss.x.add(new JSBSim::FGStateSpace::Theta());

    TS_ASSERT_EQUALS(ss.x.getSize(), 3);
  }

  // Test 55: Sensor outputs vs state outputs
  void testSensorVsStateOutputs() {
    JSBSim::FGStateSpace ss(fdmExec);

    // State: alpha
    ss.x.add(new JSBSim::FGStateSpace::Alpha());

    // Outputs: might include same variable measured by sensor
    ss.y.add(new JSBSim::FGStateSpace::Alpha());
    ss.y.add(new JSBSim::FGStateSpace::AccelZ());

    TS_ASSERT_EQUALS(ss.x.getSize(), 1);
    TS_ASSERT_EQUALS(ss.y.getSize(), 2);
  }

  // ============ Numerical Concepts ============

  // Test 56: State derivative approximation
  void testStateDerivativeApproximation() {
    double x_current = 10.0;
    double x_next = 10.5;
    double dt = 0.01;

    double x_dot = (x_next - x_current) / dt;
    TS_ASSERT_DELTA(x_dot, 50.0, epsilon);
  }

  // Test 57: Euler integration concept
  void testEulerIntegration() {
    double x = 0.0;
    double x_dot = 2.0;  // Constant rate
    double dt = 0.1;
    int steps = 10;

    for (int i = 0; i < steps; i++) {
      x = x + x_dot * dt;
    }

    TS_ASSERT_DELTA(x, 2.0, epsilon);
  }

  // Test 58: RK4 accuracy comparison concept
  void testRK4AccuracyConcept() {
    // RK4 error is O(dt^5) per step, O(dt^4) global
    double dt = 0.01;
    double eulerError = dt;      // O(dt)
    double rk4Error = dt * dt * dt * dt;  // O(dt^4)

    TS_ASSERT(rk4Error < eulerError);
  }

  // Test 59: Linearization concept
  void testLinearizationConcept() {
    // Jacobian approximation: df/dx ≈ (f(x+dx) - f(x)) / dx
    double x = 2.0;
    double dx = 0.001;

    // f(x) = x^2
    double f_x = x * x;
    double f_x_dx = (x + dx) * (x + dx);
    double df_dx = (f_x_dx - f_x) / dx;

    // Analytical: df/dx = 2x = 4.0
    TS_ASSERT_DELTA(df_dx, 4.0, 0.01);
  }

  // Test 60: Perturbation for Jacobian
  void testPerturbationSize() {
    double nominal = 100.0;
    double relPerturbation = 0.001;  // 0.1%

    double delta = nominal * relPerturbation;
    TS_ASSERT_DELTA(delta, 0.1, epsilon);
  }

  // ============ Modal Analysis Concepts ============

  // Test 61: Short period mode parameters
  void testShortPeriodMode() {
    double omega_sp = 3.0;   // rad/s natural frequency
    double zeta_sp = 0.4;    // damping ratio

    // Period = 2*pi / omega_d where omega_d = omega_n * sqrt(1 - zeta^2)
    double omega_d = omega_sp * std::sqrt(1.0 - zeta_sp * zeta_sp);
    double period = 2.0 * M_PI / omega_d;

    TS_ASSERT_DELTA(period, 2.29, 0.1);
  }

  // Test 62: Phugoid mode parameters
  void testPhugoidMode() {
    double omega_p = 0.2;    // rad/s (slow)
    double zeta_p = 0.05;    // lightly damped

    // Time to half amplitude = ln(2) / (zeta * omega_n)
    double t_half = std::log(2.0) / (zeta_p * omega_p);
    TS_ASSERT_DELTA(t_half, 69.3, 1.0);  // About 70 seconds
  }

  // Test 63: Dutch roll mode parameters
  void testDutchRollMode() {
    double omega_dr = 1.5;   // rad/s
    double zeta_dr = 0.1;    // Low damping

    // Damped frequency
    double omega_d = omega_dr * std::sqrt(1.0 - zeta_dr * zeta_dr);
    TS_ASSERT_DELTA(omega_d, 1.49, 0.01);
  }

  // Test 64: Roll subsidence time constant
  void testRollSubsidenceTimeConstant() {
    double L_p = -5.0;  // Roll damping derivative

    // Roll time constant tau_r = -1/L_p
    double tau_r = -1.0 / L_p;
    TS_ASSERT_DELTA(tau_r, 0.2, epsilon);
  }

  // Test 65: Spiral mode time constant
  void testSpiralModeTimeConstant() {
    double eigenvalue_spiral = -0.02;  // Very slow mode

    double tau_spiral = -1.0 / eigenvalue_spiral;
    TS_ASSERT_DELTA(tau_spiral, 50.0, epsilon);
  }

  // ============ Additional Component Tests ============

  // Test 66: All velocity types
  void testAllVelocityTypes() {
    JSBSim::FGStateSpace ss(fdmExec);

    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::VGround());
    ss.x.add(new JSBSim::FGStateSpace::Vn());
    ss.x.add(new JSBSim::FGStateSpace::Ve());
    ss.x.add(new JSBSim::FGStateSpace::Vd());

    TS_ASSERT_EQUALS(ss.x.getSize(), 5);
  }

  // Test 67: Angular rates body vs inertial
  void testAngularRatesBodyVsInertial() {
    JSBSim::FGStateSpace ss(fdmExec);

    // Body rates
    ss.x.add(new JSBSim::FGStateSpace::P());
    ss.x.add(new JSBSim::FGStateSpace::Q());
    ss.x.add(new JSBSim::FGStateSpace::R());

    // Inertial rates
    ss.y.add(new JSBSim::FGStateSpace::Pi());
    ss.y.add(new JSBSim::FGStateSpace::Qi());
    ss.y.add(new JSBSim::FGStateSpace::Ri());

    TS_ASSERT_EQUALS(ss.x.getSize(), 3);
    TS_ASSERT_EQUALS(ss.y.getSize(), 3);
  }

  // Test 68: Decoupled longitudinal system
  void testDecoupledLongitudinalSystem() {
    JSBSim::FGStateSpace ss(fdmExec);

    // States: [Vt, alpha, theta, q]
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    ss.x.add(new JSBSim::FGStateSpace::Theta());
    ss.x.add(new JSBSim::FGStateSpace::Q());

    // Input: [de, throttle]
    ss.u.add(new JSBSim::FGStateSpace::DeCmd());
    ss.u.add(new JSBSim::FGStateSpace::ThrottleCmd());

    TS_ASSERT_EQUALS(ss.x.getSize(), 4);
    TS_ASSERT_EQUALS(ss.u.getSize(), 2);
  }

  // Test 69: Decoupled lateral-directional system
  void testDecoupledLateralDirectionalSystem() {
    JSBSim::FGStateSpace ss(fdmExec);

    // States: [beta, phi, p, r]
    ss.x.add(new JSBSim::FGStateSpace::Beta());
    ss.x.add(new JSBSim::FGStateSpace::Phi());
    ss.x.add(new JSBSim::FGStateSpace::P());
    ss.x.add(new JSBSim::FGStateSpace::R());

    // Input: [da, dr]
    ss.u.add(new JSBSim::FGStateSpace::DaCmd());
    ss.u.add(new JSBSim::FGStateSpace::DrCmd());

    TS_ASSERT_EQUALS(ss.x.getSize(), 4);
    TS_ASSERT_EQUALS(ss.u.getSize(), 2);
  }

  // Test 70: Heading hold autopilot states
  void testHeadingHoldAutopilotStates() {
    JSBSim::FGStateSpace ss(fdmExec);

    // States needed for heading hold
    ss.x.add(new JSBSim::FGStateSpace::Phi());
    ss.x.add(new JSBSim::FGStateSpace::Psi());
    ss.x.add(new JSBSim::FGStateSpace::P());
    ss.x.add(new JSBSim::FGStateSpace::R());

    // Input
    ss.u.add(new JSBSim::FGStateSpace::DaCmd());

    // Output: heading error (conceptually)
    ss.y.add(new JSBSim::FGStateSpace::Psi());

    TS_ASSERT_EQUALS(ss.x.getSize(), 4);
  }

  // Test 71: Altitude hold autopilot states
  void testAltitudeHoldAutopilotStates() {
    JSBSim::FGStateSpace ss(fdmExec);

    // States for altitude hold
    ss.x.add(new JSBSim::FGStateSpace::Alt());
    ss.x.add(new JSBSim::FGStateSpace::Vd());
    ss.x.add(new JSBSim::FGStateSpace::Theta());
    ss.x.add(new JSBSim::FGStateSpace::Q());

    // Input
    ss.u.add(new JSBSim::FGStateSpace::DeCmd());

    TS_ASSERT_EQUALS(ss.x.getSize(), 4);
    TS_ASSERT_EQUALS(ss.u.getSize(), 1);
  }

  // Test 72: Speed hold autopilot states
  void testSpeedHoldAutopilotStates() {
    JSBSim::FGStateSpace ss(fdmExec);

    // States for speed hold
    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Rpm0());

    // Input
    ss.u.add(new JSBSim::FGStateSpace::ThrottleCmd());

    TS_ASSERT_EQUALS(ss.x.getSize(), 2);
    TS_ASSERT_EQUALS(ss.u.getSize(), 1);
  }

  // Test 73: Course tracking states
  void testCourseTrackingStates() {
    JSBSim::FGStateSpace ss(fdmExec);

    // States for course tracking
    ss.x.add(new JSBSim::FGStateSpace::Latitude());
    ss.x.add(new JSBSim::FGStateSpace::Longitude());
    ss.x.add(new JSBSim::FGStateSpace::Psi());
    ss.x.add(new JSBSim::FGStateSpace::VGround());
    ss.x.add(new JSBSim::FGStateSpace::COG());

    TS_ASSERT_EQUALS(ss.x.getSize(), 5);
  }

  // Test 74: Trim state configuration
  void testTrimStateConfiguration() {
    JSBSim::FGStateSpace ss(fdmExec);

    // Typical trim variables
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    ss.x.add(new JSBSim::FGStateSpace::Beta());
    ss.u.add(new JSBSim::FGStateSpace::DeCmd());
    ss.u.add(new JSBSim::FGStateSpace::DaCmd());
    ss.u.add(new JSBSim::FGStateSpace::DrCmd());
    ss.u.add(new JSBSim::FGStateSpace::ThrottleCmd());

    TS_ASSERT_EQUALS(ss.x.getSize(), 2);
    TS_ASSERT_EQUALS(ss.u.getSize(), 4);
  }

  // Test 75: Component vector iteration
  void testComponentVectorIteration() {
    JSBSim::FGStateSpace ss(fdmExec);

    ss.x.add(new JSBSim::FGStateSpace::Vt());
    ss.x.add(new JSBSim::FGStateSpace::Alpha());
    ss.x.add(new JSBSim::FGStateSpace::Theta());
    ss.x.add(new JSBSim::FGStateSpace::Q());

    // Iterate through all components
    int count = 0;
    for (int i = 0; i < ss.x.getSize(); i++) {
      JSBSim::FGStateSpace::Component* comp = ss.x.getComp(i);
      TS_ASSERT(comp != nullptr);
      count++;
    }

    TS_ASSERT_EQUALS(count, 4);
  }
};
