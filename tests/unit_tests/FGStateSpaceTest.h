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
};
