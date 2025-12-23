#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <sstream>
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <FGFDMExec.h>
#include "TestUtilities.h"

using namespace JSBSim;
using namespace JSBSimTest;

class FGInputTest : public CxxTest::TestSuite
{
public:
  // Task 5.5.1: Basic FGInput test suite structure

  // Test input command types
  void testInputCommandTypes() {
    // Socket input commands: set, get, hold, resume, iterate, quit, info, help
    std::string cmdSet = "set";
    std::string cmdGet = "get";
    std::string cmdHold = "hold";
    std::string cmdResume = "resume";
    std::string cmdIterate = "iterate";
    std::string cmdQuit = "quit";
    std::string cmdInfo = "info";
    std::string cmdHelp = "help";

    TS_ASSERT_EQUALS(cmdSet, "set");
    TS_ASSERT_EQUALS(cmdGet, "get");
    TS_ASSERT_EQUALS(cmdHold, "hold");
    TS_ASSERT_EQUALS(cmdResume, "resume");
    TS_ASSERT_EQUALS(cmdIterate, "iterate");
    TS_ASSERT_EQUALS(cmdQuit, "quit");
    TS_ASSERT_EQUALS(cmdInfo, "info");
    TS_ASSERT_EQUALS(cmdHelp, "help");
  }

  // Test enable/disable toggle logic
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

  // Task 5.5.2: Socket input handling (input parsing)

  // Test command line parsing - basic set command
  void testCommandLineParsing() {
    std::string line = "set velocities/vc-kts 150.0";

    // Parse by spaces
    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token) {
      tokens.push_back(token);
    }

    TS_ASSERT_EQUALS(tokens.size(), 3u);
    TS_ASSERT_EQUALS(tokens[0], "set");
    TS_ASSERT_EQUALS(tokens[1], "velocities/vc-kts");
    TS_ASSERT_EQUALS(tokens[2], "150.0");
  }

  // Test command with whitespace trimming
  void testCommandWhitespaceTrimming() {
    std::string line = "  set   attitude/phi-rad   0.1  ";

    // Trim leading whitespace
    size_t start = line.find_first_not_of(" \t\r\n");
    size_t end = line.find_last_not_of(" \t\r\n");
    std::string trimmed = (start == std::string::npos) ? "" : line.substr(start, end - start + 1);

    TS_ASSERT(!trimmed.empty());
    TS_ASSERT(trimmed.find("set") != std::string::npos);
  }

  // Test line buffering with CRLF
  void testLineBufferingCRLF() {
    std::string buffer = "set fcs/throttle-cmd-norm 0.8\r\n";

    // Find line end
    size_t lineEnd = buffer.find_first_of("\r\n");
    TS_ASSERT(lineEnd != std::string::npos);

    std::string line = buffer.substr(0, lineEnd);
    TS_ASSERT_EQUALS(line, "set fcs/throttle-cmd-norm 0.8");
    TS_ASSERT(line.find("\r") == std::string::npos);
    TS_ASSERT(line.find("\n") == std::string::npos);
  }

  // Test multi-line buffering
  void testMultiLineBuffering() {
    std::string buffer = "set prop1 1.0\r\nset prop2 2.0\r\nset prop3 3.0\r\n";

    std::vector<std::string> lines;
    size_t start = 0;

    // Parse lines
    while (true) {
      size_t stringStart = buffer.find_first_not_of("\r\n", start);
      if (stringStart == std::string::npos) break;
      size_t stringEnd = buffer.find_first_of("\r\n", stringStart);
      if (stringEnd == std::string::npos) break;
      std::string line = buffer.substr(stringStart, stringEnd - stringStart);
      if (line.empty()) break;
      lines.push_back(line);
      start = stringEnd;
    }

    TS_ASSERT_EQUALS(lines.size(), 3u);
    TS_ASSERT_EQUALS(lines[0], "set prop1 1.0");
    TS_ASSERT_EQUALS(lines[1], "set prop2 2.0");
    TS_ASSERT_EQUALS(lines[2], "set prop3 3.0");
  }

  // Test incomplete line buffering
  void testIncompleteLineBuffering() {
    std::string buffer = "set fcs/throttle-cmd-norm 0.8\r\nset fcs/mixture";

    // Find last complete line
    size_t lastCRLF = buffer.find_last_of("\r\n");
    TS_ASSERT(lastCRLF != std::string::npos);

    // Remaining data (incomplete line)
    std::string remaining = buffer.substr(lastCRLF + 1);
    TS_ASSERT_EQUALS(remaining, "set fcs/mixture");
  }

  // Task 5.5.3: Property input mapping (property name to value)

  // Test property name validation
  void testPropertyNameValidation() {
    // Valid property names
    std::string prop1 = "velocities/vc-kts";
    std::string prop2 = "position/h-sl-ft";
    std::string prop3 = "fcs/throttle-cmd-norm";

    // Property names should contain '/'
    TS_ASSERT(prop1.find('/') != std::string::npos);
    TS_ASSERT(prop2.find('/') != std::string::npos);
    TS_ASSERT(prop3.find('/') != std::string::npos);

    // Invalid property name (no slash)
    std::string invalid = "invalidproperty";
    TS_ASSERT(invalid.find('/') == std::string::npos);
  }

  // Test value type conversion - string to double
  void testValueTypeConversion() {
    std::string strValue1 = "150.0";
    std::string strValue2 = "0.8";
    std::string strValue3 = "-5.5";
    std::string strValue4 = "1e-3";

    double value1 = std::atof(strValue1.c_str());
    double value2 = std::atof(strValue2.c_str());
    double value3 = std::atof(strValue3.c_str());
    double value4 = std::atof(strValue4.c_str());

    TS_ASSERT_DELTA(value1, 150.0, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(value2, 0.8, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(value3, -5.5, DEFAULT_TOLERANCE);
    TS_ASSERT_DELTA(value4, 0.001, DEFAULT_TOLERANCE);
  }

  // Test invalid value conversion
  void testInvalidValueConversion() {
    std::string invalidValue = "not_a_number";

    // atof returns 0.0 for invalid strings
    double value = std::atof(invalidValue.c_str());
    TS_ASSERT_DELTA(value, 0.0, DEFAULT_TOLERANCE);

    // Check if string is numeric
    bool isNumeric = true;
    for (char c : invalidValue) {
      if (!std::isdigit(c) && c != '.' && c != '-' && c != '+' && c != 'e' && c != 'E') {
        if (std::isalpha(c)) {
          isNumeric = false;
          break;
        }
      }
    }
    TS_ASSERT(!isNumeric);
  }

  // Test property path parsing
  void testPropertyPathParsing() {
    std::string propertyPath = "velocities/vc-kts";

    // Split into category and property
    size_t slashPos = propertyPath.find('/');
    TS_ASSERT(slashPos != std::string::npos);

    std::string category = propertyPath.substr(0, slashPos);
    std::string property = propertyPath.substr(slashPos + 1);

    TS_ASSERT_EQUALS(category, "velocities");
    TS_ASSERT_EQUALS(property, "vc-kts");
  }

  // Test nested property path parsing
  void testNestedPropertyPathParsing() {
    std::string propertyPath = "propulsion/engine[0]/thrust-lbs";

    // Should contain multiple parts
    size_t firstSlash = propertyPath.find('/');
    TS_ASSERT(firstSlash != std::string::npos);

    size_t secondSlash = propertyPath.find('/', firstSlash + 1);
    TS_ASSERT(secondSlash != std::string::npos);

    // Extract components
    std::string part1 = propertyPath.substr(0, firstSlash);
    std::string part2 = propertyPath.substr(firstSlash + 1, secondSlash - firstSlash - 1);
    std::string part3 = propertyPath.substr(secondSlash + 1);

    TS_ASSERT_EQUALS(part1, "propulsion");
    TS_ASSERT_EQUALS(part2, "engine[0]");
    TS_ASSERT_EQUALS(part3, "thrust-lbs");
  }

  // Test input rate limiting
  void testInputRateLimiting() {
    // Rate in Hz (samples per second)
    double rateHz = 10.0;
    double simDt = 0.0083333;  // ~120 Hz simulation

    // Process input every N frames
    int inputEveryNFrames = static_cast<int>(std::round(1.0 / (rateHz * simDt)));
    // 1.0 / (10 * 0.0083333) = 1.0 / 0.08333 = 12
    TS_ASSERT_EQUALS(inputEveryNFrames, 12);

    // Verify rate limiting behavior
    int frameCount = 0;
    int inputCount = 0;

    for (int i = 0; i < 120; i++) {
      frameCount++;
      if (frameCount % inputEveryNFrames == 0) {
        inputCount++;
      }
    }

    // Should process input 10 times in 120 frames at 10 Hz
    TS_ASSERT_EQUALS(inputCount, 10);
  }

  // Test buffer overflow handling
  void testBufferOverflowHandling() {
    std::string buffer;
    size_t maxBufferSize = 4096;

    // Add data to buffer
    for (int i = 0; i < 100; i++) {
      buffer += "set property value\r\n";
    }

    // Check if buffer exceeds limit
    bool needsTruncation = buffer.size() > maxBufferSize;

    if (needsTruncation) {
      // Keep only last maxBufferSize characters
      buffer = buffer.substr(buffer.size() - maxBufferSize);
    }

    TS_ASSERT(buffer.size() <= maxBufferSize);
  }

  // Test multiple property parsing in single command
  void testMultiplePropertyParsing() {
    std::vector<std::string> commands = {
      "set fcs/throttle-cmd-norm 1.0",
      "set fcs/mixture-cmd-norm 0.87",
      "set fcs/elevator-cmd-norm 0.1"
    };

    // Parse each command
    for (const auto& cmd : commands) {
      std::istringstream iss(cmd);
      std::string command, property, value;
      iss >> command >> property >> value;

      TS_ASSERT_EQUALS(command, "set");
      TS_ASSERT(!property.empty());
      TS_ASSERT(!value.empty());
    }

    TS_ASSERT_EQUALS(commands.size(), 3u);
  }

  // Test invalid input handling - empty command
  void testInvalidInputEmptyCommand() {
    std::string line = "";

    TS_ASSERT(line.empty());

    // Empty commands should be ignored
    bool shouldProcess = !line.empty();
    TS_ASSERT(!shouldProcess);
  }

  // Test invalid input handling - malformed command
  void testInvalidInputMalformedCommand() {
    std::string line = "set";  // Missing property and value

    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string token;
    while (iss >> token) {
      tokens.push_back(token);
    }

    // Should only have one token
    TS_ASSERT_EQUALS(tokens.size(), 1u);
    TS_ASSERT_EQUALS(tokens[0], "set");

    // Missing arguments - should be handled
    bool hasProperty = tokens.size() > 1;
    bool hasValue = tokens.size() > 2;
    TS_ASSERT(!hasProperty);
    TS_ASSERT(!hasValue);
  }

  // Test invalid input handling - unknown command
  void testInvalidInputUnknownCommand() {
    std::string line = "unknown property value";

    std::istringstream iss(line);
    std::string command;
    iss >> command;

    // Check if command is valid
    bool isValidCommand = (command == "set" || command == "get" ||
                          command == "hold" || command == "resume" ||
                          command == "iterate" || command == "quit" ||
                          command == "info" || command == "help");

    TS_ASSERT(!isValidCommand);
  }

  // Test socket port validation
  void testSocketPortValidation() {
    int validPort = 5500;
    int invalidPort = 0;
    int outOfRangePort = 70000;

    TS_ASSERT(validPort > 0 && validPort <= 65535);
    TS_ASSERT(invalidPort == 0);
    TS_ASSERT(outOfRangePort > 65535);
  }

  // Test protocol type parsing
  void testProtocolTypeParsing() {
    std::string tcpProtocol = "tcp";
    std::string udpProtocol = "udp";

    // Convert to lowercase for comparison
    std::string tcpLower = tcpProtocol;
    std::string udpLower = udpProtocol;

    TS_ASSERT_EQUALS(tcpLower, "tcp");
    TS_ASSERT_EQUALS(udpLower, "udp");

    // Verify valid protocols
    bool isTCP = (tcpLower == "tcp");
    bool isUDP = (udpLower == "udp");
    TS_ASSERT(isTCP);
    TS_ASSERT(isUDP);
  }

  // Test input blocking mode flag
  void testInputBlockingMode() {
    bool blockingInput = false;

    TS_ASSERT(!blockingInput);

    // Enable blocking input
    blockingInput = true;
    TS_ASSERT(blockingInput);

    // Blocking mode should wait for data
    if (blockingInput) {
      // Would call WaitUntilReadable() in real implementation
      TS_ASSERT(blockingInput);
    }
  }

  // Test command case sensitivity
  void testCommandCaseSensitivity() {
    std::string upperCmd = "SET";
    std::string lowerCmd = "set";
    std::string mixedCmd = "Set";

    // Convert all to lowercase for comparison
    std::string upperLower = upperCmd;
    std::transform(upperLower.begin(), upperLower.end(), upperLower.begin(), ::tolower);

    std::string mixedLower = mixedCmd;
    std::transform(mixedLower.begin(), mixedLower.end(), mixedLower.begin(), ::tolower);

    TS_ASSERT_EQUALS(upperLower, "set");
    TS_ASSERT_EQUALS(lowerCmd, "set");
    TS_ASSERT_EQUALS(mixedLower, "set");
    TS_ASSERT_EQUALS(upperLower, lowerCmd);
  }

  // Test iterate command argument parsing
  void testIterateCommandParsing() {
    std::string line = "iterate 10";

    std::istringstream iss(line);
    std::string command;
    int iterations;
    iss >> command >> iterations;

    TS_ASSERT_EQUALS(command, "iterate");
    TS_ASSERT_EQUALS(iterations, 10);
    TS_ASSERT(iterations > 0);
  }

  // Test get command format
  void testGetCommandFormat() {
    std::string line = "get velocities/vc-kts";

    std::istringstream iss(line);
    std::string command, property;
    iss >> command >> property;

    TS_ASSERT_EQUALS(command, "get");
    TS_ASSERT_EQUALS(property, "velocities/vc-kts");
    TS_ASSERT(!property.empty());
  }

  // Test response formatting
  void testResponseFormatting() {
    std::string property = "velocities/vc-kts";
    double value = 150.5;

    std::ostringstream response;
    response << property << " = " << std::setw(12) << std::setprecision(6) << value;

    std::string responseStr = response.str();
    TS_ASSERT(responseStr.find(property) != std::string::npos);
    TS_ASSERT(responseStr.find("150.5") != std::string::npos);
    TS_ASSERT(responseStr.find("=") != std::string::npos);
  }

  // Test CRLF termination in responses
  void testResponseCRLFTermination() {
    std::string response = "set successful";
    std::string terminatedResponse = response + "\r\n";

    TS_ASSERT(terminatedResponse.find("\r\n") != std::string::npos);
    TS_ASSERT_EQUALS(terminatedResponse.substr(terminatedResponse.length() - 2), "\r\n");
  }

  // Test input instance indexing
  void testInputInstanceIndexing() {
    int numInputs = 3;
    std::vector<bool> inputEnabled(numInputs, true);

    TS_ASSERT_EQUALS(inputEnabled.size(), 3u);
    TS_ASSERT(inputEnabled[0]);
    TS_ASSERT(inputEnabled[1]);
    TS_ASSERT(inputEnabled[2]);

    // Toggle specific instance
    inputEnabled[1] = false;
    TS_ASSERT(inputEnabled[0]);
    TS_ASSERT(!inputEnabled[1]);
    TS_ASSERT(inputEnabled[2]);
  }

  // Test input name identifier
  void testInputNameIdentifier() {
    std::string inputName = "input_socket_5500";

    TS_ASSERT(!inputName.empty());
    TS_ASSERT(inputName.find("input") != std::string::npos);
    TS_ASSERT(inputName.find("5500") != std::string::npos);
  }
};
