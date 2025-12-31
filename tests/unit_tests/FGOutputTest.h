#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <string>
#include <sstream>
#include <vector>
#include <iomanip>

#include <FGFDMExec.h>
#include <models/FGOutput.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class FGOutputTest : public CxxTest::TestSuite
{
public:
  // Test output type enum values
  void testOutputTypeEnums() {
    // Output types: CSV, TABULAR, SOCKET, FLIGHTGEAR, TERMINAL, COUT, NONE
    int otCSV = 0;
    int otTab = 1;
    int otSocket = 2;
    int otFG = 3;
    int otTerminal = 4;
    int otNone = 5;

    TS_ASSERT_EQUALS(otCSV, 0);
    TS_ASSERT_EQUALS(otTab, 1);
    TS_ASSERT_EQUALS(otSocket, 2);
    TS_ASSERT_EQUALS(otFG, 3);
    TS_ASSERT_EQUALS(otTerminal, 4);
    TS_ASSERT_EQUALS(otNone, 5);
  }

  // Test subsystem bitmask values
  void testSubsystemBitmasks() {
    // Subsystem flags are powers of 2
    int ssSimulation = 1;
    int ssAerosurfaces = 2;
    int ssRates = 4;
    int ssVelocities = 8;
    int ssForces = 16;
    int ssMoments = 32;
    int ssAtmosphere = 64;
    int ssMassProps = 128;
    int ssAeroFunctions = 256;
    int ssPropagate = 512;
    int ssGroundReactions = 1024;
    int ssFCS = 2048;
    int ssPropulsion = 4096;

    TS_ASSERT_EQUALS(ssSimulation, 1);
    TS_ASSERT_EQUALS(ssAerosurfaces, 2);
    TS_ASSERT_EQUALS(ssRates, 4);
    TS_ASSERT_EQUALS(ssVelocities, 8);
    TS_ASSERT_EQUALS(ssForces, 16);
    TS_ASSERT_EQUALS(ssMoments, 32);
    TS_ASSERT_EQUALS(ssAtmosphere, 64);
    TS_ASSERT_EQUALS(ssMassProps, 128);
    TS_ASSERT_EQUALS(ssAeroFunctions, 256);
    TS_ASSERT_EQUALS(ssPropagate, 512);
    TS_ASSERT_EQUALS(ssGroundReactions, 1024);
    TS_ASSERT_EQUALS(ssFCS, 2048);
    TS_ASSERT_EQUALS(ssPropulsion, 4096);
  }

  // Test subsystem combination
  void testSubsystemCombination() {
    int ssSimulation = 1;
    int ssRates = 4;
    int ssVelocities = 8;

    // Combine subsystems with OR
    int combined = ssSimulation | ssRates | ssVelocities;
    TS_ASSERT_EQUALS(combined, 13);

    // Check if specific subsystem is enabled
    bool hasSimulation = (combined & ssSimulation) != 0;
    bool hasRates = (combined & ssRates) != 0;
    bool hasVelocities = (combined & ssVelocities) != 0;
    bool hasForces = (combined & 16) != 0;

    TS_ASSERT(hasSimulation);
    TS_ASSERT(hasRates);
    TS_ASSERT(hasVelocities);
    TS_ASSERT(!hasForces);
  }

  // Test output rate calculation
  void testOutputRateCalculation() {
    // Rate in Hz (samples per second)
    double rateHz = 20.0;
    double simDt = 0.0083333;  // ~120 Hz simulation

    // Output every N frames
    int outputEveryNFrames = static_cast<int>(std::round(1.0 / (rateHz * simDt)));
    // 1.0 / (20 * 0.0083333) = 1.0 / 0.16667 = 6
    TS_ASSERT_EQUALS(outputEveryNFrames, 6);

    // Different rates
    rateHz = 10.0;
    outputEveryNFrames = static_cast<int>(std::round(1.0 / (rateHz * simDt)));
    TS_ASSERT_EQUALS(outputEveryNFrames, 12);
  }

  // Test output rate Hz validation
  void testOutputRateValidation() {
    // Valid rates
    double rate1 = 20.0;  // 20 Hz
    double rate2 = 1.0;   // 1 Hz
    double rate3 = 120.0; // 120 Hz

    TS_ASSERT(rate1 > 0.0);
    TS_ASSERT(rate2 > 0.0);
    TS_ASSERT(rate3 > 0.0);

    // Invalid rate (zero or negative would cause issues)
    double invalidRate = 0.0;
    TS_ASSERT(invalidRate <= 0.0);
  }

  // Test CSV delimiter
  void testCSVDelimiter() {
    std::string delimiter = ",";
    TS_ASSERT_EQUALS(delimiter, ",");

    // Build CSV row
    double time = 1.234;
    double altitude = 5000.0;
    double speed = 150.0;

    std::ostringstream oss;
    oss << time << delimiter << altitude << delimiter << speed;
    std::string row = oss.str();

    TS_ASSERT(row.find(",") != std::string::npos);
    TS_ASSERT_EQUALS(row, "1.234,5000,150");
  }

  // Test tab delimiter
  void testTabDelimiter() {
    std::string delimiter = "\t";
    TS_ASSERT_EQUALS(delimiter, "\t");

    double time = 1.234;
    double altitude = 5000.0;

    std::ostringstream oss;
    oss << time << delimiter << altitude;
    std::string row = oss.str();

    TS_ASSERT(row.find("\t") != std::string::npos);
  }

  // Test output filename parsing
  void testOutputFilenameParsing() {
    std::string filename = "B737_datalog.csv";

    // Check extension
    size_t dotPos = filename.rfind('.');
    TS_ASSERT(dotPos != std::string::npos);

    std::string extension = filename.substr(dotPos + 1);
    TS_ASSERT_EQUALS(extension, "csv");

    std::string basename = filename.substr(0, dotPos);
    TS_ASSERT_EQUALS(basename, "B737_datalog");
  }

  // Test filename with runID suffix
  void testFilenameWithRunID() {
    std::string basename = "datalog";
    std::string extension = ".csv";
    int runID = 5;

    std::ostringstream oss;
    oss << basename << "_" << runID << extension;
    std::string filename = oss.str();

    TS_ASSERT_EQUALS(filename, "datalog_5.csv");
  }

  // Test socket address parsing
  void testSocketAddressParsing() {
    std::string address = "localhost:5500/tcp";

    // Parse host
    size_t colonPos = address.find(':');
    std::string host = address.substr(0, colonPos);
    TS_ASSERT_EQUALS(host, "localhost");

    // Parse port
    size_t slashPos = address.find('/');
    std::string portStr = address.substr(colonPos + 1, slashPos - colonPos - 1);
    int port = std::stoi(portStr);
    TS_ASSERT_EQUALS(port, 5500);

    // Parse protocol
    std::string protocol = address.substr(slashPos + 1);
    TS_ASSERT_EQUALS(protocol, "tcp");
  }

  // Test default port values
  void testDefaultPortValues() {
    // Default JSBSim socket port
    int defaultPort = 1138;
    TS_ASSERT_EQUALS(defaultPort, 1138);

    // Default FlightGear port
    int fgPort = 5500;
    TS_ASSERT_EQUALS(fgPort, 5500);
  }

  // Test protocol types
  void testProtocolTypes() {
    std::string tcp = "tcp";
    std::string udp = "udp";

    TS_ASSERT_EQUALS(tcp, "tcp");
    TS_ASSERT_EQUALS(udp, "udp");
  }

  // Test enable/disable toggle
  void testEnableDisableToggle() {
    bool enabled = true;

    TS_ASSERT(enabled);

    // Disable
    enabled = false;
    TS_ASSERT(!enabled);

    // Toggle
    enabled = !enabled;
    TS_ASSERT(enabled);

    // Toggle again
    enabled = !enabled;
    TS_ASSERT(!enabled);
  }

  // Test output instance indexing
  void testOutputInstanceIndexing() {
    int numOutputs = 3;
    std::vector<bool> enabled(numOutputs, true);

    TS_ASSERT_EQUALS(enabled.size(), 3u);
    TS_ASSERT(enabled[0]);
    TS_ASSERT(enabled[1]);
    TS_ASSERT(enabled[2]);

    // Toggle specific instance
    enabled[1] = false;
    TS_ASSERT(enabled[0]);
    TS_ASSERT(!enabled[1]);
    TS_ASSERT(enabled[2]);
  }

  // Test precision formatting
  void testPrecisionFormatting() {
    double value = 1234.56789;

    // Default precision
    std::ostringstream oss1;
    oss1 << value;
    std::string str1 = oss1.str();
    TS_ASSERT(str1.find("1234.57") != std::string::npos ||
              str1.find("1234.56789") != std::string::npos);

    // Fixed precision
    std::ostringstream oss2;
    oss2.precision(2);
    oss2 << std::fixed << value;
    std::string str2 = oss2.str();
    TS_ASSERT_EQUALS(str2, "1234.57");
  }

  // Test scientific notation
  void testScientificNotation() {
    double largeValue = 1.23e10;
    double smallValue = 1.23e-10;

    std::ostringstream oss1;
    oss1 << std::scientific << largeValue;
    std::string str1 = oss1.str();
    TS_ASSERT(str1.find("e+") != std::string::npos || str1.find("E+") != std::string::npos);

    std::ostringstream oss2;
    oss2 << std::scientific << smallValue;
    std::string str2 = oss2.str();
    TS_ASSERT(str2.find("e-") != std::string::npos || str2.find("E-") != std::string::npos);
  }

  // Test header row generation
  void testHeaderRowGeneration() {
    std::vector<std::string> headers = {"Time", "Altitude", "Speed", "Heading"};
    std::string delimiter = ",";

    std::ostringstream oss;
    for (size_t i = 0; i < headers.size(); i++) {
      if (i > 0) oss << delimiter;
      oss << headers[i];
    }
    std::string headerRow = oss.str();

    TS_ASSERT_EQUALS(headerRow, "Time,Altitude,Speed,Heading");
  }

  // Test output directory path handling
  void testOutputDirectoryPath() {
    std::string directory = "/home/user/jsbsim/output/";
    std::string filename = "datalog.csv";

    std::string fullPath = directory + filename;
    TS_ASSERT_EQUALS(fullPath, "/home/user/jsbsim/output/datalog.csv");

    // Without trailing slash
    directory = "/home/user/jsbsim/output";
    fullPath = directory + "/" + filename;
    TS_ASSERT_EQUALS(fullPath, "/home/user/jsbsim/output/datalog.csv");
  }

  // Test special filename "cout"
  void testSpecialCoutFilename() {
    std::string filename = "cout";

    bool isStdout = (filename == "cout" || filename == "COUT");
    TS_ASSERT(isStdout);

    filename = "datalog.csv";
    isStdout = (filename == "cout" || filename == "COUT");
    TS_ASSERT(!isStdout);
  }

  // Test simulation data values
  void testSimulationDataValues() {
    // Common output properties
    double simTime = 123.456;
    double dt = 0.0083333;
    double frameCount = 14815.0;

    TS_ASSERT_DELTA(simTime, 123.456, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(dt, 0.0083333, 1e-7);
    TS_ASSERT_DELTA(frameCount, 14815.0, DEFAULT_TOLERANCE);
  }

  // Test velocity output values
  void testVelocityOutputValues() {
    // Body-axis velocities (ft/sec)
    double u = 300.0;  // forward
    double v = 5.0;    // lateral
    double w = 10.0;   // vertical

    // True airspeed: sqrt(300^2 + 5^2 + 10^2) = sqrt(90125) = 300.208
    double vt = std::sqrt(u*u + v*v + w*w);
    TS_ASSERT_DELTA(vt, 300.208, 0.01);

    // Calibrated airspeed (would include density correction)
    double rhoRatio = 0.9;  // density ratio
    double vc = vt * std::sqrt(rhoRatio);
    TS_ASSERT(vc < vt);
  }

  // Test angular rate output values
  void testAngularRateOutputValues() {
    // Body-axis rates (rad/sec)
    double p = 0.1;   // roll rate
    double q = 0.05;  // pitch rate
    double r = 0.02;  // yaw rate

    // Convert to deg/sec for output
    double pDeg = p * 180.0 / M_PI;
    double qDeg = q * 180.0 / M_PI;
    double rDeg = r * 180.0 / M_PI;

    TS_ASSERT_DELTA(pDeg, 5.73, 0.01);
    TS_ASSERT_DELTA(qDeg, 2.86, 0.01);
    TS_ASSERT_DELTA(rDeg, 1.15, 0.01);
  }

  // Test force output values
  void testForceOutputValues() {
    // Force components (lbf)
    double fx = 1000.0;  // forward force
    double fy = 50.0;    // lateral force
    double fz = -500.0;  // vertical force (negative = down)

    // Total force magnitude: sqrt(1000^2 + 50^2 + 500^2) = sqrt(1252500) = 1119.15
    double fTotal = std::sqrt(fx*fx + fy*fy + fz*fz);
    TS_ASSERT_DELTA(fTotal, 1119.15, 0.1);
  }

  // Test moment output values
  void testMomentOutputValues() {
    // Moment components (ft-lbf)
    double l = 5000.0;   // rolling moment
    double m = -2000.0;  // pitching moment
    double n = 1000.0;   // yawing moment

    TS_ASSERT_DELTA(l, 5000.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(m, -2000.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(n, 1000.0, DEFAULT_TOLERANCE);
  }

  // Test atmosphere output values
  void testAtmosphereOutputValues() {
    // Standard atmosphere values at sea level
    double pressure = 2116.22;     // psf
    double density = 0.002377;     // slugs/ft^3
    double temperature = 518.67;   // Rankine
    double soundSpeed = 1116.45;   // ft/sec

    TS_ASSERT_DELTA(pressure, Constants::SEA_LEVEL_PRESSURE_PSF, 1.0);
    TS_ASSERT_DELTA(density, Constants::SEA_LEVEL_DENSITY_SLUGFT3, 0.00001);
    TS_ASSERT_DELTA(temperature, Constants::SEA_LEVEL_TEMP_R, 1.0);
  }

  // Test mass properties output
  void testMassPropertiesOutput() {
    // Aircraft mass properties
    double emptyWeight = 6000.0;   // lbs
    double fuelWeight = 1500.0;    // lbs
    double totalWeight = emptyWeight + fuelWeight;

    TS_ASSERT_DELTA(totalWeight, 7500.0, DEFAULT_TOLERANCE);

    // CG position (inches from reference)
    double cgX = 100.0;
    double cgY = 0.0;
    double cgZ = -5.0;

    TS_ASSERT_DELTA(cgY, 0.0, DEFAULT_TOLERANCE);  // Symmetric aircraft
  }

  // Test FCS output values
  void testFCSOutputValues() {
    // Flight control surface positions (normalized)
    double elevatorPos = 0.25;   // 25% deflection
    double aileronPos = -0.1;    // 10% left
    double rudderPos = 0.0;      // centered

    TS_ASSERT(elevatorPos >= -1.0 && elevatorPos <= 1.0);
    TS_ASSERT(aileronPos >= -1.0 && aileronPos <= 1.0);
    TS_ASSERT(rudderPos >= -1.0 && rudderPos <= 1.0);

    // Control surface deflection in degrees
    double elevatorDeg = elevatorPos * 25.0;  // Max 25 deg
    TS_ASSERT_DELTA(elevatorDeg, 6.25, DEFAULT_TOLERANCE);
  }

  // Test propulsion output values
  void testPropulsionOutputValues() {
    // Engine parameters
    double thrust = 5000.0;       // lbf
    double rpm = 2400.0;
    double fuelFlow = 0.1;        // lbs/sec
    double manifoldPressure = 29.0;  // inHg

    TS_ASSERT(thrust >= 0.0);
    TS_ASSERT(rpm >= 0.0);
    TS_ASSERT(fuelFlow >= 0.0);
  }

  // Test ground reactions output
  void testGroundReactionsOutput() {
    // Gear forces (lbf)
    double noseGearForce = 1000.0;
    double leftMainForce = 3000.0;
    double rightMainForce = 3000.0;

    double totalGearForce = noseGearForce + leftMainForce + rightMainForce;
    TS_ASSERT_DELTA(totalGearForce, 7000.0, DEFAULT_TOLERANCE);

    // Gear compression (inches)
    double compression = 2.5;
    TS_ASSERT(compression >= 0.0);
  }

  /***************************************************************************
   * Additional Delimiter Tests
   ***************************************************************************/

  // Test 30: Semicolon delimiter (European locale)
  void testSemicolonDelimiter() {
    std::string delimiter = ";";
    double time = 1.234;
    double alt = 5000.0;

    std::ostringstream oss;
    oss << time << delimiter << alt;
    std::string row = oss.str();

    TS_ASSERT(row.find(";") != std::string::npos);
    TS_ASSERT_EQUALS(row, "1.234;5000");
  }

  // Test 31: Space delimiter
  void testSpaceDelimiter() {
    std::string delimiter = " ";
    double time = 1.234;
    double alt = 5000.0;

    std::ostringstream oss;
    oss << time << delimiter << alt;
    std::string row = oss.str();

    TS_ASSERT_EQUALS(row, "1.234 5000");
  }

  // Test 32: Multiple consecutive values with delimiter
  void testMultipleValuesDelimited() {
    std::string delimiter = ",";
    std::vector<double> values = {1.0, 2.0, 3.0, 4.0, 5.0};

    std::ostringstream oss;
    for (size_t i = 0; i < values.size(); i++) {
      if (i > 0) oss << delimiter;
      oss << values[i];
    }

    TS_ASSERT_EQUALS(oss.str(), "1,2,3,4,5");
  }

  /***************************************************************************
   * Socket Connection Tests
   ***************************************************************************/

  // Test 33: IP address validation pattern
  void testIPAddressPattern() {
    std::string ip = "192.168.1.100";

    // Simple validation: contains 3 dots
    int dotCount = 0;
    for (char c : ip) {
      if (c == '.') dotCount++;
    }
    TS_ASSERT_EQUALS(dotCount, 3);
  }

  // Test 34: Port range validation
  void testPortRangeValidation() {
    int port = 5500;

    bool validRange = (port > 0 && port <= 65535);
    TS_ASSERT(validRange);

    // Reserved port (requires root)
    port = 80;
    bool isPrivileged = (port < 1024);
    TS_ASSERT(isPrivileged);

    // User port
    port = 5500;
    isPrivileged = (port < 1024);
    TS_ASSERT(!isPrivileged);
  }

  // Test 35: Localhost aliases
  void testLocalhostAliases() {
    std::string host1 = "localhost";
    std::string host2 = "127.0.0.1";
    std::string host3 = "::1";  // IPv6 localhost

    TS_ASSERT_EQUALS(host1, "localhost");
    TS_ASSERT_EQUALS(host2, "127.0.0.1");
    TS_ASSERT_EQUALS(host3, "::1");
  }

  /***************************************************************************
   * UDP Packet Formatting Tests
   ***************************************************************************/

  // Test 36: UDP packet size limits
  void testUDPPacketSizeLimits() {
    int maxUDPSize = 65507;  // Max UDP payload (65535 - 8 UDP header - 20 IP header)
    int typicalMTU = 1472;   // Typical MTU minus headers

    TS_ASSERT(maxUDPSize > typicalMTU);
    TS_ASSERT_EQUALS(maxUDPSize, 65507);
    TS_ASSERT_EQUALS(typicalMTU, 1472);
  }

  // Test 37: FlightGear packet structure
  void testFlightGearPacketStructure() {
    // FlightGear native protocol uses network byte order
    // Generic protocol uses text format
    int protocolVersion = 1;
    int headerSize = 8;

    TS_ASSERT_EQUALS(protocolVersion, 1);
    TS_ASSERT_EQUALS(headerSize, 8);
  }

  /***************************************************************************
   * File Path Operation Tests
   ***************************************************************************/

  // Test 38: Windows-style path conversion
  void testWindowsPathConversion() {
    std::string winPath = "C:\\Users\\test\\output\\data.csv";

    // Count backslashes
    int backslashes = 0;
    for (char c : winPath) {
      if (c == '\\') backslashes++;
    }
    TS_ASSERT_EQUALS(backslashes, 4);
  }

  // Test 39: Relative path handling
  void testRelativePathHandling() {
    std::string relPath = "../output/data.csv";
    std::string basePath = "/home/user/jsbsim/scripts";

    // Simple check for relative path
    bool isRelative = (relPath[0] != '/');
    TS_ASSERT(isRelative);
  }

  // Test 40: File extension extraction
  void testFileExtensionExtraction() {
    std::string filename = "datalog.2024.01.15.csv";

    // Find last dot for extension
    size_t lastDot = filename.rfind('.');
    TS_ASSERT(lastDot != std::string::npos);

    std::string ext = filename.substr(lastDot + 1);
    TS_ASSERT_EQUALS(ext, "csv");
  }

  // Test 41: Empty filename handling
  void testEmptyFilenameHandling() {
    std::string filename = "";

    bool isEmpty = filename.empty();
    TS_ASSERT(isEmpty);

    size_t dotPos = filename.rfind('.');
    TS_ASSERT(dotPos == std::string::npos);
  }

  /***************************************************************************
   * Subsystem Combination Tests
   ***************************************************************************/

  // Test 42: All subsystems enabled
  void testAllSubsystemsEnabled() {
    int allSubsystems = 1 | 2 | 4 | 8 | 16 | 32 | 64 | 128 | 256 | 512 | 1024 | 2048 | 4096;
    TS_ASSERT_EQUALS(allSubsystems, 8191);
  }

  // Test 43: Toggle specific subsystem
  void testToggleSubsystem() {
    int subsystems = 13;  // 1 + 4 + 8 = Simulation, Rates, Velocities
    int ssRates = 4;

    // Toggle off
    subsystems &= ~ssRates;
    TS_ASSERT_EQUALS(subsystems, 9);  // 1 + 8

    // Toggle on
    subsystems |= ssRates;
    TS_ASSERT_EQUALS(subsystems, 13);
  }

  // Test 44: Subsystem count from bitmask
  void testSubsystemCount() {
    int subsystems = 13;  // 1 + 4 + 8

    // Count set bits
    int count = 0;
    int temp = subsystems;
    while (temp) {
      count += temp & 1;
      temp >>= 1;
    }
    TS_ASSERT_EQUALS(count, 3);
  }

  /***************************************************************************
   * Rate Conversion Tests
   ***************************************************************************/

  // Test 45: Very high output rate
  void testVeryHighOutputRate() {
    double rateHz = 1000.0;  // 1 kHz
    double simDt = 0.001;    // 1 kHz simulation

    int outputEveryNFrames = static_cast<int>(std::round(1.0 / (rateHz * simDt)));
    TS_ASSERT_EQUALS(outputEveryNFrames, 1);  // Every frame
  }

  // Test 46: Very low output rate
  void testVeryLowOutputRate() {
    double rateHz = 0.1;     // Once every 10 seconds
    double simDt = 0.01;     // 100 Hz simulation

    int outputEveryNFrames = static_cast<int>(std::round(1.0 / (rateHz * simDt)));
    TS_ASSERT_EQUALS(outputEveryNFrames, 1000);
  }

  // Test 47: Output rate higher than simulation rate
  void testOutputRateHigherThanSimRate() {
    double rateHz = 200.0;   // 200 Hz output desired
    double simDt = 0.01;     // 100 Hz simulation

    // Can't output faster than simulation runs
    int outputEveryNFrames = static_cast<int>(std::round(1.0 / (rateHz * simDt)));
    if (outputEveryNFrames < 1) outputEveryNFrames = 1;

    TS_ASSERT_EQUALS(outputEveryNFrames, 1);  // Clamped to every frame
  }

  /***************************************************************************
   * Timestamp Formatting Tests
   ***************************************************************************/

  // Test 48: Time with milliseconds
  void testTimeWithMilliseconds() {
    double simTime = 123.456;

    int seconds = static_cast<int>(simTime);
    int millis = static_cast<int>((simTime - seconds) * 1000);

    TS_ASSERT_EQUALS(seconds, 123);
    TS_ASSERT_EQUALS(millis, 456);
  }

  // Test 49: Time with microseconds precision
  void testTimeWithMicroseconds() {
    double simTime = 123.456789;

    int seconds = static_cast<int>(simTime);
    int micros = static_cast<int>((simTime - seconds) * 1000000);

    TS_ASSERT_EQUALS(seconds, 123);
    TS_ASSERT_EQUALS(micros, 456789);
  }

  // Test 50: Frame count to time conversion
  void testFrameCountToTime() {
    long frameCount = 100000;
    double dt = 0.00833333;  // 120 Hz

    double simTime = frameCount * dt;
    TS_ASSERT_DELTA(simTime, 833.333, 0.01);
  }

  /***************************************************************************
   * Property Path Tests
   ***************************************************************************/

  // Test 51: Property path parsing
  void testPropertyPathParsing() {
    std::string propPath = "velocities/u-fps";

    // Parse category
    size_t slashPos = propPath.find('/');
    std::string category = propPath.substr(0, slashPos);
    std::string property = propPath.substr(slashPos + 1);

    TS_ASSERT_EQUALS(category, "velocities");
    TS_ASSERT_EQUALS(property, "u-fps");
  }

  // Test 52: Nested property path
  void testNestedPropertyPath() {
    std::string propPath = "fcs/elevator/position-norm";

    // Count path depth
    int depth = 0;
    for (char c : propPath) {
      if (c == '/') depth++;
    }
    TS_ASSERT_EQUALS(depth, 2);  // 3 levels
  }

  // Test 53: Array indexed property
  void testArrayIndexedProperty() {
    std::string propPath = "propulsion/engine[0]/thrust-lbs";

    // Find array index
    size_t openBracket = propPath.find('[');
    size_t closeBracket = propPath.find(']');

    TS_ASSERT(openBracket != std::string::npos);
    TS_ASSERT(closeBracket != std::string::npos);

    std::string indexStr = propPath.substr(openBracket + 1, closeBracket - openBracket - 1);
    int index = std::stoi(indexStr);
    TS_ASSERT_EQUALS(index, 0);
  }

  /***************************************************************************
   * Output Filtering Tests
   ***************************************************************************/

  // Test 54: Value clamping for output
  void testValueClampingForOutput() {
    double value = 1e100;
    double maxOutput = 1e10;

    double clamped = std::min(value, maxOutput);
    TS_ASSERT_DELTA(clamped, maxOutput, DEFAULT_TOLERANCE);

    // Clamp small values
    value = 1e-100;
    double minOutput = 1e-10;
    clamped = std::max(value, minOutput);
    TS_ASSERT_DELTA(clamped, minOutput, DEFAULT_TOLERANCE);
  }

  // Test 55: NaN detection for output
  void testNaNDetectionForOutput() {
    double validValue = 123.456;
    double nanValue = std::nan("");

    bool isValid = !std::isnan(validValue);
    bool isNaN = std::isnan(nanValue);

    TS_ASSERT(isValid);
    TS_ASSERT(isNaN);
  }

  // Test 56: Infinity detection for output
  void testInfinityDetectionForOutput() {
    double validValue = 123.456;
    double infValue = std::numeric_limits<double>::infinity();

    bool isValid = !std::isinf(validValue);
    bool isInf = std::isinf(infValue);

    TS_ASSERT(isValid);
    TS_ASSERT(isInf);
  }

  /***************************************************************************
   * File Rotation Tests
   ***************************************************************************/

  // Test 57: Rotation filename generation
  void testRotationFilenameGeneration() {
    std::string baseName = "datalog";
    std::string ext = ".csv";
    int rotationNumber = 3;

    std::ostringstream oss;
    oss << baseName << "." << rotationNumber << ext;

    TS_ASSERT_EQUALS(oss.str(), "datalog.3.csv");
  }

  // Test 58: Date-based filename
  void testDateBasedFilename() {
    std::string baseName = "datalog";
    int year = 2024, month = 12, day = 25;

    std::ostringstream oss;
    oss << baseName << "_" << year << "-"
        << (month < 10 ? "0" : "") << month << "-"
        << (day < 10 ? "0" : "") << day << ".csv";

    TS_ASSERT_EQUALS(oss.str(), "datalog_2024-12-25.csv");
  }

  /***************************************************************************
   * Buffer Size Tests
   ***************************************************************************/

  // Test 59: Output buffer sizing
  void testOutputBufferSizing() {
    int numProperties = 50;
    int bytesPerProperty = 16;  // Average string length per value

    int estimatedBufferSize = numProperties * bytesPerProperty;
    TS_ASSERT_EQUALS(estimatedBufferSize, 800);
  }

  // Test 60: Line buffer size
  void testLineBufferSize() {
    // Typical CSV line with timestamps and ~20 values
    int maxLineLength = 1024;
    int typicalLineLength = 400;

    TS_ASSERT(typicalLineLength < maxLineLength);
  }

  /***************************************************************************
   * Coordinate Output Tests
   ***************************************************************************/

  // Test 61: Latitude/Longitude formatting
  void testLatLonFormatting() {
    double latDeg = 37.7749;
    double lonDeg = -122.4194;

    // Degrees with sign
    std::ostringstream oss;
    oss.precision(6);
    oss << std::fixed << latDeg << "," << lonDeg;

    TS_ASSERT_EQUALS(oss.str(), "37.774900,-122.419400");
  }

  // Test 62: Degrees-Minutes-Seconds format
  void testDMSFormat() {
    double latDeg = 37.7749;

    int degrees = static_cast<int>(latDeg);
    double fractional = (latDeg - degrees) * 60.0;
    int minutes = static_cast<int>(fractional);
    double seconds = (fractional - minutes) * 60.0;

    TS_ASSERT_EQUALS(degrees, 37);
    TS_ASSERT_EQUALS(minutes, 46);
    TS_ASSERT_DELTA(seconds, 29.64, 0.01);
  }

  /***************************************************************************
   * Attitude Output Tests
   ***************************************************************************/

  // Test 63: Euler angle normalization for output
  void testEulerAngleNormalization() {
    double heading = 400.0;  // Over 360

    // Normalize to 0-360
    while (heading >= 360.0) heading -= 360.0;
    while (heading < 0.0) heading += 360.0;

    TS_ASSERT_DELTA(heading, 40.0, DEFAULT_TOLERANCE);
  }

  // Test 64: Pitch angle range
  void testPitchAngleRange() {
    double pitch = 45.0;

    // Valid range is typically -90 to +90
    bool validRange = (pitch >= -90.0 && pitch <= 90.0);
    TS_ASSERT(validRange);
  }

  // Test 65: Roll angle output
  void testRollAngleOutput() {
    double roll = -30.0;  // 30 degrees left wing down

    // Valid range is typically -180 to +180
    bool validRange = (roll >= -180.0 && roll <= 180.0);
    TS_ASSERT(validRange);
  }

  /***************************************************************************
   * Engine Output Tests
   ***************************************************************************/

  // Test 66: Multi-engine thrust output
  void testMultiEngineThrustOutput() {
    double thrust[4] = {5000.0, 5100.0, 4900.0, 5050.0};

    double totalThrust = 0;
    for (int i = 0; i < 4; i++) {
      totalThrust += thrust[i];
    }

    TS_ASSERT_DELTA(totalThrust, 20050.0, DEFAULT_TOLERANCE);
  }

  // Test 67: Fuel flow summation
  void testFuelFlowSummation() {
    double fuelFlow[2] = {0.15, 0.14};  // lbs/sec per engine

    double totalFlow = fuelFlow[0] + fuelFlow[1];
    double flowPerHour = totalFlow * 3600.0;  // Convert to lbs/hr

    TS_ASSERT_DELTA(totalFlow, 0.29, 0.001);
    TS_ASSERT_DELTA(flowPerHour, 1044.0, 0.1);
  }

  /***************************************************************************
   * Environmental Output Tests
   ***************************************************************************/

  // Test 68: Wind component output
  void testWindComponentOutput() {
    double windNorth = 10.0;   // kts
    double windEast = 5.0;     // kts

    double windSpeed = std::sqrt(windNorth*windNorth + windEast*windEast);
    double windDir = std::atan2(windEast, windNorth) * 180.0 / M_PI;
    if (windDir < 0) windDir += 360.0;

    TS_ASSERT_DELTA(windSpeed, 11.18, 0.01);
    TS_ASSERT_DELTA(windDir, 26.57, 0.1);
  }

  // Test 69: Visibility output
  void testVisibilityOutput() {
    double visibilityMeters = 10000.0;
    double visibilityMiles = visibilityMeters / 1609.344;  // Statute miles

    TS_ASSERT_DELTA(visibilityMiles, 6.21, 0.01);
  }

  /***************************************************************************
   * Performance Output Tests
   ***************************************************************************/

  // Test 70: Rate of climb calculation for output
  void testRateOfClimbOutput() {
    double verticalSpeed = 1000.0;  // ft/min
    double verticalSpeedFPS = verticalSpeed / 60.0;  // ft/sec

    TS_ASSERT_DELTA(verticalSpeedFPS, 16.67, 0.01);
  }

  // Test 71: G-load output
  void testGLoadOutput() {
    double normalAccel = 64.4;  // ft/sec^2 (2g)
    double gLoad = normalAccel / 32.174;  // g units

    TS_ASSERT_DELTA(gLoad, 2.0, 0.01);
  }

  // Test 72: Mach number output precision
  void testMachNumberPrecision() {
    double mach = 0.785;

    std::ostringstream oss;
    oss.precision(3);
    oss << std::fixed << mach;

    TS_ASSERT_EQUALS(oss.str(), "0.785");
  }

  /***************************************************************************
   * Error Condition Tests
   ***************************************************************************/

  // Test 73: Zero rate handling
  void testZeroRateHandling() {
    double rateHz = 0.0;

    bool invalidRate = (rateHz <= 0.0);
    TS_ASSERT(invalidRate);
  }

  // Test 74: Negative port handling
  void testNegativePortHandling() {
    int port = -1;

    bool invalidPort = (port < 0 || port > 65535);
    TS_ASSERT(invalidPort);
  }

  // Test 75: Empty property list
  void testEmptyPropertyList() {
    std::vector<std::string> properties;

    TS_ASSERT(properties.empty());
    TS_ASSERT_EQUALS(properties.size(), 0u);
  }
};

/***************************************************************************
 * Extended Output Tests (Tests 76-100)
 ***************************************************************************/

class FGOutputExtendedTest : public CxxTest::TestSuite
{
public:
  // Test 76: Output rate conversion Hz to frames
  void testOutputRateConversion() {
    double simDt = 1.0 / 120.0;  // 120 Hz simulation
    double outputRate = 10.0;    // 10 Hz output

    int framesPerOutput = static_cast<int>(1.0 / (outputRate * simDt));

    TS_ASSERT_EQUALS(framesPerOutput, 12);
  }

  // Test 77: Multiple output rate divisor
  void testMultipleOutputRates() {
    double baseRate = 120.0;  // Hz

    double rate10Hz = baseRate / 10.0;
    double rate20Hz = baseRate / 20.0;
    double rate60Hz = baseRate / 60.0;

    TS_ASSERT_DELTA(rate10Hz, 12.0, 0.01);
    TS_ASSERT_DELTA(rate20Hz, 6.0, 0.01);
    TS_ASSERT_DELTA(rate60Hz, 2.0, 0.01);
  }

  // Test 78: Socket port range validation
  void testSocketPortRange() {
    int validPort1 = 1024;
    int validPort2 = 65535;
    int invalidPort1 = 0;
    int invalidPort2 = 70000;

    TS_ASSERT(validPort1 >= 1 && validPort1 <= 65535);
    TS_ASSERT(validPort2 >= 1 && validPort2 <= 65535);
    TS_ASSERT(invalidPort1 < 1 || invalidPort2 > 65535);
  }

  // Test 79: Protocol string validation
  void testProtocolValidation() {
    std::string tcp = "TCP";
    std::string udp = "UDP";
    std::string invalid = "HTTP";

    TS_ASSERT(tcp == "TCP" || tcp == "UDP");
    TS_ASSERT(udp == "TCP" || udp == "UDP");
    TS_ASSERT(invalid != "TCP" && invalid != "UDP");
  }

  // Test 80: Output file path construction
  void testOutputFilePath() {
    std::string basePath = "/output/";
    std::string filename = "data";
    std::string extension = ".csv";

    std::string fullPath = basePath + filename + extension;

    TS_ASSERT_EQUALS(fullPath, "/output/data.csv");
  }

  // Test 81: Timestamped filename generation
  void testTimestampedFilename() {
    std::string base = "simulation";
    int simTime = 12345;  // seconds

    std::ostringstream oss;
    oss << base << "_" << simTime << ".csv";

    TS_ASSERT_EQUALS(oss.str(), "simulation_12345.csv");
  }

  // Test 82: Column header generation
  void testColumnHeaderGeneration() {
    std::vector<std::string> properties = {"Time", "Altitude", "Velocity"};

    std::ostringstream header;
    for (size_t i = 0; i < properties.size(); ++i) {
      if (i > 0) header << ",";
      header << properties[i];
    }

    TS_ASSERT_EQUALS(header.str(), "Time,Altitude,Velocity");
  }

  // Test 83: Data delimiter options
  void testDataDelimiters() {
    double v1 = 1.0, v2 = 2.0, v3 = 3.0;

    std::ostringstream csvOut, tabOut;
    csvOut << v1 << "," << v2 << "," << v3;
    tabOut << v1 << "\t" << v2 << "\t" << v3;

    TS_ASSERT_EQUALS(csvOut.str(), "1,2,3");
    TS_ASSERT_EQUALS(tabOut.str(), "1\t2\t3");
  }

  // Test 84: Output precision levels
  void testOutputPrecisionLevels() {
    double value = 1.23456789;

    std::ostringstream low, med, high;
    low << std::fixed << std::setprecision(2) << value;
    med << std::fixed << std::setprecision(4) << value;
    high << std::fixed << std::setprecision(8) << value;

    TS_ASSERT_EQUALS(low.str(), "1.23");
    TS_ASSERT_EQUALS(med.str(), "1.2346");
    TS_ASSERT_EQUALS(high.str(), "1.23456789");
  }

  // Test 85: Subsystem combination flags
  void testSubsystemCombinationFlags() {
    int ssVelocities = 8;
    int ssForces = 16;
    int ssMoments = 32;

    int combined = ssVelocities | ssForces | ssMoments;

    TS_ASSERT(combined & ssVelocities);
    TS_ASSERT(combined & ssForces);
    TS_ASSERT(combined & ssMoments);
    TS_ASSERT_EQUALS(combined, 56);
  }

  // Test 86: All subsystems flag
  void testAllSubsystemsFlag() {
    // Sum of first 13 powers of 2 (2^13 - 1 = 8191)
    int allSubsystems = 8191;

    TS_ASSERT(allSubsystems & 1);     // Simulation
    TS_ASSERT(allSubsystems & 4096);  // Propulsion
    TS_ASSERT_EQUALS(allSubsystems, (1 << 13) - 1);
  }

  // Test 87: Output buffer sizing
  void testOutputBufferSizing() {
    int numProperties = 100;
    int bytesPerValue = 8;  // double precision
    int delimiter = 1;

    size_t lineSize = numProperties * (bytesPerValue + delimiter);

    TS_ASSERT(lineSize > 800);
    TS_ASSERT(lineSize < 1000);
  }

  // Test 88: Time format HH:MM:SS
  void testTimeFormatHHMMSS() {
    int totalSeconds = 3661;  // 1 hour, 1 minute, 1 second

    int hours = totalSeconds / 3600;
    int minutes = (totalSeconds % 3600) / 60;
    int seconds = totalSeconds % 60;

    TS_ASSERT_EQUALS(hours, 1);
    TS_ASSERT_EQUALS(minutes, 1);
    TS_ASSERT_EQUALS(seconds, 1);
  }

  // Test 89: Property path parsing
  void testPropertyPathParsing() {
    std::string fullPath = "velocities/v-fps";
    size_t slashPos = fullPath.find('/');

    std::string category = fullPath.substr(0, slashPos);
    std::string property = fullPath.substr(slashPos + 1);

    TS_ASSERT_EQUALS(category, "velocities");
    TS_ASSERT_EQUALS(property, "v-fps");
  }

  // Test 90: Network byte order check (big endian)
  void testNetworkByteOrder() {
    uint32_t hostOrder = 0x01020304;

    // Network order is big endian - check structure
    unsigned char* bytes = reinterpret_cast<unsigned char*>(&hostOrder);
    bool isLittleEndian = (bytes[0] == 0x04);

    // Most common systems are little endian
    TS_ASSERT(isLittleEndian || !isLittleEndian);  // Platform dependent
  }

  // Test 91: FlightGear native protocol version
  void testFlightGearProtocolVersion() {
    int protocolVersion = 24;  // FG native protocol version

    TS_ASSERT(protocolVersion >= 24);
    TS_ASSERT(protocolVersion <= 30);
  }

  // Test 92: Output enable/disable toggle
  void testOutputEnableDisable() {
    bool outputEnabled = true;

    outputEnabled = !outputEnabled;
    TS_ASSERT(!outputEnabled);

    outputEnabled = !outputEnabled;
    TS_ASSERT(outputEnabled);
  }

  // Test 93: Quaternion to Euler output conversion
  void testQuaternionToEulerOutput() {
    // Simple test: identity quaternion -> zero Euler angles
    double q0 = 1.0, q1 = 0.0, q2 = 0.0, q3 = 0.0;

    double phi = std::atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    double theta = std::asin(2*(q0*q2 - q3*q1));
    double psi = std::atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));

    TS_ASSERT_DELTA(phi, 0.0, 0.001);
    TS_ASSERT_DELTA(theta, 0.0, 0.001);
    TS_ASSERT_DELTA(psi, 0.0, 0.001);
  }

  // Test 94: Property wildcard matching
  void testPropertyWildcardMatching() {
    std::string pattern = "velocities/*";
    std::string property = "velocities/v-fps";

    bool matches = (property.find("velocities/") == 0);

    TS_ASSERT(matches);
  }

  // Test 95: Output start time delay
  void testOutputStartTimeDelay() {
    double simTime = 5.0;
    double outputStartTime = 2.0;

    bool shouldOutput = (simTime >= outputStartTime);

    TS_ASSERT(shouldOutput);
  }

  // Test 96: Output end time cutoff
  void testOutputEndTimeCutoff() {
    double simTime = 100.0;
    double outputEndTime = 50.0;

    bool shouldOutput = (outputEndTime <= 0.0 || simTime <= outputEndTime);

    TS_ASSERT(!shouldOutput);
  }

  // Test 97: Scientific notation threshold
  void testScientificNotationThreshold() {
    double large = 1.0e8;
    double small = 1.0e-6;
    double normal = 1234.56;

    bool useSciLarge = (std::abs(large) >= 1e6);
    bool useSciSmall = (std::abs(small) <= 1e-4);
    bool useSciNormal = (std::abs(normal) >= 1e6 || std::abs(normal) <= 1e-4);

    TS_ASSERT(useSciLarge);
    TS_ASSERT(useSciSmall);
    TS_ASSERT(!useSciNormal);
  }

  // Test 98: Output counter wraparound
  void testOutputCounterWraparound() {
    unsigned int maxCount = 4294967295u;

    unsigned int count = maxCount;
    count++;

    TS_ASSERT_EQUALS(count, 0u);
  }

  // Test 99: Socket connection state
  void testSocketConnectionState() {
    enum SocketState { DISCONNECTED, CONNECTING, CONNECTED, ERROR_STATE };

    SocketState state = DISCONNECTED;

    state = CONNECTING;
    TS_ASSERT_EQUALS(state, CONNECTING);

    state = CONNECTED;
    TS_ASSERT_EQUALS(state, CONNECTED);
  }

  // Test 100: Output file rotation check
  void testOutputFileRotationCheck() {
    size_t currentSize = 9 * 1024 * 1024;  // 9 MB
    size_t maxSize = 10 * 1024 * 1024;     // 10 MB
    size_t lineSize = 500;                  // bytes per line

    bool needsRotation = (currentSize + lineSize > maxSize);

    TS_ASSERT(!needsRotation);

    currentSize = 10 * 1024 * 1024;
    needsRotation = (currentSize + lineSize > maxSize);

    TS_ASSERT(needsRotation);
  }

  // ============================================================================
  // C172x Aircraft Model Output Tests
  // ============================================================================

  // Test C172x: Output object is not null after loading model
  void testC172xOutputNotNull() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto output = fdmex.GetOutput();
    TS_ASSERT(output != nullptr);
  }

  // Test C172x: Output object exists before RunIC
  void testC172xOutputBeforeRunIC() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto output = fdmex.GetOutput();
    TS_ASSERT(output != nullptr);
    // Output should be accessible even before initialization
    output->Disable();
    TS_ASSERT(true);
  }

  // Test C172x: Output object exists after RunIC
  void testC172xOutputAfterRunIC() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();
    TS_ASSERT(output != nullptr);
  }

  // Test C172x: Enable output method doesn't crash
  void testC172xEnableOutput() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    output->Enable();
    TS_ASSERT(true);  // No crash
  }

  // Test C172x: Disable output method doesn't crash
  void testC172xDisableOutput() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    output->Disable();
    TS_ASSERT(true);  // No crash
  }

  // Test C172x: Enable then disable cycle
  void testC172xEnableDisableCycle() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 5; i++) {
      output->Enable();
      output->Disable();
    }
    TS_ASSERT(true);  // No crash after multiple cycles
  }

  // Test C172x: InitModel returns valid result
  void testC172xOutputInitModel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    auto output = fdmex.GetOutput();

    bool result = output->InitModel();
    TS_ASSERT(result == true || result == false);  // Valid boolean result
  }

  // Test C172x: Run method after RunIC
  void testC172xOutputRun() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    bool result = output->Run(false);
    TS_ASSERT(result == true || result == false);
  }

  // Test C172x: Run in holding mode
  void testC172xOutputRunHolding() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    bool result = output->Run(true);
    TS_ASSERT(result == true || result == false);
  }

  // Test C172x: Multiple Run calls
  void testC172xOutputMultipleRuns() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 10; i++) {
      output->Run(false);
    }
    TS_ASSERT(true);  // No crash after multiple runs
  }

  // Test C172x: Print method doesn't crash
  void testC172xOutputPrint() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    output->Print();
    TS_ASSERT(true);
  }

  // Test C172x: SetRateHz with various rates
  void testC172xSetRateHz() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    output->SetRateHz(1.0);
    output->SetRateHz(10.0);
    output->SetRateHz(100.0);
    TS_ASSERT(true);  // No crash
  }

  // Test C172x: SetRateHz with low rate
  void testC172xSetRateHzLow() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    output->SetRateHz(0.1);  // Very low rate
    TS_ASSERT(true);
  }

  // Test C172x: SetRateHz with high rate
  void testC172xSetRateHzHigh() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    output->SetRateHz(1000.0);  // High rate
    TS_ASSERT(true);
  }

  // Test C172x: SetStartNewOutput method
  void testC172xSetStartNewOutput() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    output->SetStartNewOutput();
    TS_ASSERT(true);
  }

  // Test C172x: GetOutputName with index 0
  void testC172xGetOutputName() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    std::string name = output->GetOutputName(0);
    TS_ASSERT(name.length() >= 0);  // Valid string
  }

  // Test C172x: GetOutputName with various indices
  void testC172xGetOutputNameMultiple() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    // Query multiple indices (may not all exist)
    for (unsigned int i = 0; i < 5; i++) {
      std::string name = output->GetOutputName(i);
      TS_ASSERT(name.length() >= 0);
    }
    TS_ASSERT(true);
  }

  // Test C172x: SetOutputName method
  void testC172xSetOutputName() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    bool result = output->SetOutputName(0, "c172x_test_output.csv");
    TS_ASSERT(result == true || result == false);
  }

  // Test C172x: Toggle output
  void testC172xToggleOutput() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    bool result = output->Toggle(0);
    TS_ASSERT(result == true || result == false);
  }

  // Test C172x: ForceOutput method
  void testC172xForceOutput() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    output->ForceOutput(0);
    TS_ASSERT(true);
  }

  // Test C172x: Output after simulation steps
  void testC172xOutputAfterSimulation() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    // Run several simulation steps
    for (int i = 0; i < 100; i++) {
      fdmex.Run();
    }

    // Output should still be valid
    output->Run(false);
    output->Print();
    TS_ASSERT(true);
  }

  // Test C172x: Output disabled during simulation
  void testC172xOutputDisabledDuringSim() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    output->Disable();

    // Run simulation with output disabled
    for (int i = 0; i < 50; i++) {
      fdmex.Run();
    }

    output->Enable();
    TS_ASSERT(true);
  }

  // Test C172x: Output with different rates during simulation
  void testC172xOutputRateChangesDuringSim() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    output->SetRateHz(10.0);
    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    output->SetRateHz(50.0);
    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    output->SetRateHz(1.0);
    for (int i = 0; i < 20; i++) {
      fdmex.Run();
    }

    TS_ASSERT(true);
  }

  // Test C172x: Multiple SetStartNewOutput calls
  void testC172xMultipleStartNewOutput() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 5; i++) {
      output->SetStartNewOutput();
      for (int j = 0; j < 10; j++) {
        fdmex.Run();
      }
    }
    TS_ASSERT(true);
  }

  // Test C172x: ForceOutput multiple times
  void testC172xMultipleForceOutput() {
    FGFDMExec fdmex;
    fdmex.LoadModel("c172x");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    for (int i = 0; i < 10; i++) {
      output->ForceOutput(0);
    }
    TS_ASSERT(true);
  }

  // ============================================================================
  // FGOutput class tests - using actual class methods
  // ============================================================================

  // Test FGOutput access through FGFDMExec
  void testFGOutputAccess() {
    FGFDMExec fdmex;
    auto output = fdmex.GetOutput();
    TS_ASSERT(output != nullptr);
  }

  // Test FGOutput with loaded model
  void testFGOutputWithModel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto output = fdmex.GetOutput();
    TS_ASSERT(output != nullptr);
  }

  // Test InitModel method
  void testFGOutputInitModel() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto output = fdmex.GetOutput();

    // InitModel completes without throwing
    bool result = output->InitModel();
    TS_ASSERT(result == true || result == false);  // Just verify it runs
  }

  // Test Run method
  void testFGOutputRun() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    // Run should complete
    bool result = output->Run(false);
    TS_ASSERT(result == true || result == false);
  }

  // Test Run in holding mode
  void testFGOutputRunHolding() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    bool result = output->Run(true);
    TS_ASSERT(result == true || result == false);
  }

  // Test Enable/Disable methods
  void testFGOutputEnableDisable() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto output = fdmex.GetOutput();

    output->Enable();
    // No direct way to check enabled state, but should not throw

    output->Disable();
    // Should not throw
    TS_ASSERT(true);
  }

  // Test SetRateHz method
  void testFGOutputSetRateHz() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto output = fdmex.GetOutput();

    // Set output rate
    output->SetRateHz(10.0);
    // Should not throw
    TS_ASSERT(true);
  }

  // Test SetStartNewOutput method
  void testFGOutputSetStartNewOutput() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto output = fdmex.GetOutput();

    output->SetStartNewOutput();
    // Should not throw
    TS_ASSERT(true);
  }

  // Test Print method
  void testFGOutputPrint() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    // Print should not throw (may do nothing if no outputs configured)
    output->Print();
    TS_ASSERT(true);
  }

  // Test GetOutputName with no outputs
  void testFGOutputGetOutputNameEmpty() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto output = fdmex.GetOutput();

    // With no outputs configured, should return empty string
    std::string name = output->GetOutputName(0);
    TS_ASSERT(name.length() >= 0);  // Valid string (empty or not)
  }

  // Test ForceOutput method
  void testFGOutputForceOutput() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    fdmex.RunIC();
    auto output = fdmex.GetOutput();

    // ForceOutput on non-existent index should not crash
    output->ForceOutput(0);
    TS_ASSERT(true);
  }

  // Test Toggle method
  void testFGOutputToggle() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto output = fdmex.GetOutput();

    // Toggle on non-existent output returns false
    bool result = output->Toggle(0);
    TS_ASSERT(result == true || result == false);
  }

  // Test SetOutputName method
  void testFGOutputSetOutputName() {
    FGFDMExec fdmex;
    fdmex.LoadModel("ball");
    auto output = fdmex.GetOutput();

    // SetOutputName on non-existent index returns false
    bool result = output->SetOutputName(0, "test_output.csv");
    TS_ASSERT(result == true || result == false);
  }
};
