
set(CMAKE_CXX_STANDARD 17)

set(UNIT_TESTS FGColumnVector3Test
               StringUtilitiesTest
               FGJSBBaseTest
               FGMatrix33Test
               FGQuaternionTest
               FGLocationTest
               FGGroundCallbackTest
               FGInitialConditionTest
               FGInertialTest
               FGInertiaTest
               FGPropertyValueTest
               FGTableTest
               FGRealValueTest
               FGParameterTest
               FGParameterValueTest
               FGConditionTest
               FGFunctionTest
               FGPropertyManagerTest
               FGAtmosphereTest
               FGStandardAtmosphereTest
               FGAtmosphericModelTest
               FGAuxiliaryTest
               FGAirspeedTest
               FGMSISTest
               FGLogTest
               FGNelderMeadTest
               FGRungeKuttaTest
               FGAccelerationsTest
               FGAircraftTest
               FGMassBalanceTest
               FGAerodynamicsTest
               FGModelTest
               FGPropagateTest
               FGForceTest
               FGExternalForceTest
               FGThrusterTest
               FGTankTest
               FGThermodynamicsTest
               FGPIDTest
               FGSensorTest
               FGFilterTest
               FGGainTest
               FGSwitchTest
               FGDeadBandTest
               FGSummerTest
               FGAnglesTest
               FGGyroTest
               FGKinematTest
               FGAccelerometerTest
               FGMagnetometerTest
               FGWaypointTest
               FGDistributorTest
               FGLinearActuatorTest
               FGActuatorTest
               FGXMLElementTest
               FGXMLParseTest
               FGBuoyantForcesTest
               FGGroundReactionsTest
               FGLGearTest
               FGSurfaceTest
               FGEngineTest
               FGElectricTest
               FGPropellerTest
               FGScriptTest
               FGOutputTest
               FGInputTest
               FGPistonTest
               FGTurbineTest
               FGRocketTest
               FGRotorTest
               FGNozzleTest
               FGTransmissionTest
               FGTurboPropTest
               FGWindsTest
               FGTrimTest
               FGFlightPathTest
               FGCoordinatesTest
               FGStabilityTest
               FGControlLawTest
               FGAeroBodyTest)


# Fast test suite - core math and fundamental tests for quick PR checks
set(FAST_TESTS FGColumnVector3Test
               FGMatrix33Test
               FGQuaternionTest
               FGInertiaTest
               FGLocationTest
               FGRealValueTest
               FGTableTest
               FGConditionTest
               FGPropertyValueTest)

foreach(test ${UNIT_TESTS})
  cxxtest_add_test(${test}1 ${test}.cpp ${CMAKE_CURRENT_SOURCE_DIR}/${test}.h)
  target_link_libraries(${test}1 libJSBSim)
  target_include_directories(${test}1 PUBLIC ${CXXTEST_INCLUDE_DIR})
  add_coverage(${test}1)

  # Add test labels for ctest -L filtering
  # Fast tests: core math, run quickly for PR checks
  # Comprehensive tests: full suite for nightly runs
  if(${test} IN_LIST FAST_TESTS)
    set_tests_properties(${test}1 PROPERTIES LABELS "quick;unit")
  else()
    set_tests_properties(${test}1 PROPERTIES LABELS "comprehensive;unit")
  endif()
endforeach()

if(WIN32 AND BUILD_SHARED_LIBS)
  # Windows cannot locate the symbol gtd7 as it is not exported in the JSBSim
  # DLL. To keep NRLMSIS source files pristine, the option chosen is to
  # compile NRLMSIS alongside the unit test FGMSISTest1.
  set(MSIS_SOURCE_DIR ${PROJECT_SOURCE_DIR}/src/models/atmosphere/MSIS)
  target_sources(FGMSISTest1 PUBLIC
                  ${MSIS_SOURCE_DIR}/nrlmsise-00.c
                  ${MSIS_SOURCE_DIR}/nrlmsise-00_data.c)

  # Windows needs the DLL to be copied locally for unit tests to run.
  list(GET UNIT_TESTS 0 FIRST_UNIT_TEST) # Any target name will do so we just pick the first.
  add_custom_command(TARGET ${FIRST_UNIT_TEST}1 POST_BUILD
                      COMMAND ${CMAKE_COMMAND} -E copy_if_different
                      $<TARGET_FILE:libJSBSim>
                      $<TARGET_FILE_DIR:${FIRST_UNIT_TEST}1>)
endif(WIN32 AND BUILD_SHARED_LIBS)
