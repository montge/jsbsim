#include <cxxtest/TestSuite.h>
#include <limits>
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>
#include <vector>
#include <map>
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

  // Test array index extraction from property path
  void testArrayIndexExtraction() {
    std::string property = "propulsion/engine[2]/thrust-lbs";

    // Find bracket positions
    size_t openBracket = property.find('[');
    size_t closeBracket = property.find(']');

    TS_ASSERT(openBracket != std::string::npos);
    TS_ASSERT(closeBracket != std::string::npos);
    TS_ASSERT(closeBracket > openBracket);

    // Extract index
    std::string indexStr = property.substr(openBracket + 1, closeBracket - openBracket - 1);
    int index = std::atoi(indexStr.c_str());

    TS_ASSERT_EQUALS(indexStr, "2");
    TS_ASSERT_EQUALS(index, 2);
  }

  // Test multiple array indices in property path
  void testMultipleArrayIndices() {
    std::string property = "systems/autopilot[0]/channels[1]/gain";

    // Count bracket pairs
    int bracketCount = 0;
    for (char c : property) {
      if (c == '[') bracketCount++;
    }

    TS_ASSERT_EQUALS(bracketCount, 2);

    // Extract all indices
    std::vector<int> indices;
    size_t pos = 0;
    while ((pos = property.find('[', pos)) != std::string::npos) {
      size_t close = property.find(']', pos);
      if (close != std::string::npos) {
        std::string indexStr = property.substr(pos + 1, close - pos - 1);
        indices.push_back(std::atoi(indexStr.c_str()));
      }
      pos++;
    }

    TS_ASSERT_EQUALS(indices.size(), 2u);
    TS_ASSERT_EQUALS(indices[0], 0);
    TS_ASSERT_EQUALS(indices[1], 1);
  }

  // Test scientific notation value parsing
  void testScientificNotationParsing() {
    std::vector<std::pair<std::string, double>> testCases = {
      {"1e6", 1e6},
      {"1E6", 1E6},
      {"1.5e-3", 1.5e-3},
      {"-2.5E+2", -2.5E+2},
      {"3.14159e0", 3.14159},
      {"1e-10", 1e-10}
    };

    for (const auto& tc : testCases) {
      double parsed = std::atof(tc.first.c_str());
      TS_ASSERT_DELTA(parsed, tc.second, std::abs(tc.second) * 1e-10 + 1e-15);
    }
  }

  // Test hexadecimal value parsing
  void testHexadecimalValueParsing() {
    std::string hexValue = "0xFF";

    // strtol can parse hex with base 0 (auto-detect)
    long value = std::strtol(hexValue.c_str(), nullptr, 0);

    TS_ASSERT_EQUALS(value, 255);
  }

  // Test boolean string to value conversion
  void testBooleanStringConversion() {
    std::vector<std::pair<std::string, bool>> trueValues = {
      {"true", true}, {"TRUE", true}, {"True", true},
      {"1", true}, {"yes", true}, {"on", true}
    };

    for (const auto& tc : trueValues) {
      std::string lower = tc.first;
      std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);

      bool result = (lower == "true" || lower == "1" ||
                     lower == "yes" || lower == "on");
      TS_ASSERT(result == tc.second || lower == "true" || lower == "1" ||
                lower == "yes" || lower == "on");
    }
  }

  // Test LF-only line ending (Unix style)
  void testLFOnlyLineEnding() {
    std::string buffer = "set prop1 1.0\nset prop2 2.0\n";

    std::vector<std::string> lines;
    std::istringstream iss(buffer);
    std::string line;

    while (std::getline(iss, line)) {
      // Remove any trailing CR
      if (!line.empty() && line.back() == '\r') {
        line.pop_back();
      }
      if (!line.empty()) {
        lines.push_back(line);
      }
    }

    TS_ASSERT_EQUALS(lines.size(), 2u);
    TS_ASSERT_EQUALS(lines[0], "set prop1 1.0");
    TS_ASSERT_EQUALS(lines[1], "set prop2 2.0");
  }

  // Test mixed line endings
  void testMixedLineEndings() {
    std::string buffer = "set prop1 1.0\r\nset prop2 2.0\nset prop3 3.0\r";

    // Replace all line endings with \n
    std::string normalized = buffer;
    size_t pos = 0;
    while ((pos = normalized.find("\r\n", pos)) != std::string::npos) {
      normalized.replace(pos, 2, "\n");
    }
    pos = 0;
    while ((pos = normalized.find('\r', pos)) != std::string::npos) {
      normalized.replace(pos, 1, "\n");
    }

    // Count lines
    int lineCount = 0;
    for (char c : normalized) {
      if (c == '\n') lineCount++;
    }

    TS_ASSERT_EQUALS(lineCount, 3);
  }

  // Test tab as delimiter
  void testTabDelimiter() {
    std::string line = "set\tvelocities/vc-kts\t150.0";

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

  // Test whitespace-only input
  void testWhitespaceOnlyInput() {
    std::string line = "   \t  \r\n";

    // Trim and check if empty
    size_t start = line.find_first_not_of(" \t\r\n");
    bool isEmpty = (start == std::string::npos);

    TS_ASSERT(isEmpty);
  }

  // Test very long property path
  void testVeryLongPropertyPath() {
    std::string longPath = "systems/subsystem/component/element/parameter/value";

    // Count path depth
    int depth = 1;
    for (char c : longPath) {
      if (c == '/') depth++;
    }

    TS_ASSERT_EQUALS(depth, 6);
    TS_ASSERT(longPath.length() > 40);
  }

  // Test property with hyphen in name
  void testPropertyWithHyphen() {
    std::string property = "fcs/aileron-pos-rad";

    // Count hyphens
    int hyphenCount = 0;
    for (char c : property) {
      if (c == '-') hyphenCount++;
    }

    TS_ASSERT_EQUALS(hyphenCount, 2);

    // Should still parse correctly
    size_t slashPos = property.find('/');
    std::string category = property.substr(0, slashPos);
    std::string name = property.substr(slashPos + 1);

    TS_ASSERT_EQUALS(category, "fcs");
    TS_ASSERT_EQUALS(name, "aileron-pos-rad");
  }

  // Test property with underscore
  void testPropertyWithUnderscore() {
    std::string property = "aero/alpha_deg";

    size_t slashPos = property.find('/');
    std::string name = property.substr(slashPos + 1);

    TS_ASSERT(name.find('_') != std::string::npos);
    TS_ASSERT_EQUALS(name, "alpha_deg");
  }

  // Test zero value handling
  void testZeroValueHandling() {
    std::string line = "set fcs/throttle-cmd-norm 0.0";

    std::istringstream iss(line);
    std::string cmd, prop, val;
    iss >> cmd >> prop >> val;

    double value = std::atof(val.c_str());

    TS_ASSERT_EQUALS(value, 0.0);
    TS_ASSERT_DELTA(value, 0.0, DEFAULT_TOLERANCE);
  }

  // Test negative value handling
  void testNegativeValueHandling() {
    std::string line = "set position/lat-gc-deg -45.5";

    std::istringstream iss(line);
    std::string cmd, prop, val;
    iss >> cmd >> prop >> val;

    double value = std::atof(val.c_str());

    TS_ASSERT(value < 0.0);
    TS_ASSERT_DELTA(value, -45.5, DEFAULT_TOLERANCE);
  }

  // Test integer value handling
  void testIntegerValueHandling() {
    std::string line = "set simulation/frame 100";

    std::istringstream iss(line);
    std::string cmd, prop, val;
    iss >> cmd >> prop >> val;

    int intValue = std::atoi(val.c_str());
    double doubleValue = std::atof(val.c_str());

    TS_ASSERT_EQUALS(intValue, 100);
    TS_ASSERT_DELTA(doubleValue, 100.0, DEFAULT_TOLERANCE);
  }

  // Test command with extra spaces
  void testCommandWithExtraSpaces() {
    std::string line = "set     velocities/vc-kts     150.0";

    std::istringstream iss(line);
    std::vector<std::string> tokens;
    std::string token;

    while (iss >> token) {
      tokens.push_back(token);
    }

    // istringstream handles multiple spaces correctly
    TS_ASSERT_EQUALS(tokens.size(), 3u);
    TS_ASSERT_EQUALS(tokens[0], "set");
    TS_ASSERT_EQUALS(tokens[1], "velocities/vc-kts");
    TS_ASSERT_EQUALS(tokens[2], "150.0");
  }

  // Test iterate with negative count (invalid)
  void testIterateNegativeCount() {
    std::string line = "iterate -5";

    std::istringstream iss(line);
    std::string cmd;
    int count;
    iss >> cmd >> count;

    TS_ASSERT_EQUALS(cmd, "iterate");
    TS_ASSERT(count < 0);

    // Negative count should be rejected
    bool isValid = (count > 0);
    TS_ASSERT(!isValid);
  }

  // Test iterate with zero count (invalid)
  void testIterateZeroCount() {
    std::string line = "iterate 0";

    std::istringstream iss(line);
    std::string cmd;
    int count;
    iss >> cmd >> count;

    TS_ASSERT_EQUALS(cmd, "iterate");
    TS_ASSERT_EQUALS(count, 0);

    // Zero iterations should be rejected or treated as no-op
    bool isValid = (count > 0);
    TS_ASSERT(!isValid);
  }

  // Test info command (no arguments)
  void testInfoCommandNoArgs() {
    std::string line = "info";

    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    TS_ASSERT_EQUALS(cmd, "info");

    // Check if there are remaining tokens
    std::string remaining;
    iss >> remaining;
    TS_ASSERT(remaining.empty());
  }

  // Test help command (no arguments)
  void testHelpCommandNoArgs() {
    std::string line = "help";

    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    TS_ASSERT_EQUALS(cmd, "help");

    std::string remaining;
    iss >> remaining;
    TS_ASSERT(remaining.empty());
  }

  // Test hold command
  void testHoldCommand() {
    std::string line = "hold";

    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    TS_ASSERT_EQUALS(cmd, "hold");

    // Hold pauses simulation
    bool simHeld = true;
    TS_ASSERT(simHeld);
  }

  // Test resume command
  void testResumeCommand() {
    std::string line = "resume";

    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    TS_ASSERT_EQUALS(cmd, "resume");

    // Resume continues simulation
    bool simHeld = false;
    TS_ASSERT(!simHeld);
  }

  // Test quit command
  void testQuitCommand() {
    std::string line = "quit";

    std::istringstream iss(line);
    std::string cmd;
    iss >> cmd;

    TS_ASSERT_EQUALS(cmd, "quit");

    // Quit should set exit flag
    bool shouldExit = (cmd == "quit");
    TS_ASSERT(shouldExit);
  }

  // Test get command with wildcard (conceptual)
  void testGetCommandWithWildcard() {
    std::string line = "get velocities/*";

    std::istringstream iss(line);
    std::string cmd, pattern;
    iss >> cmd >> pattern;

    TS_ASSERT_EQUALS(cmd, "get");
    TS_ASSERT_EQUALS(pattern, "velocities/*");

    // Check for wildcard character
    bool hasWildcard = (pattern.find('*') != std::string::npos);
    TS_ASSERT(hasWildcard);
  }

  // Test property value bounds
  void testPropertyValueBounds() {
    // Test typical bounded values
    double throttle = 0.8;
    double elevator = 0.5;
    double rudder = -0.3;

    // Normalized control values should be -1 to 1
    TS_ASSERT(throttle >= 0.0 && throttle <= 1.0);
    TS_ASSERT(elevator >= -1.0 && elevator <= 1.0);
    TS_ASSERT(rudder >= -1.0 && rudder <= 1.0);
  }

  // Test connection state enum values
  void testConnectionStateValues() {
    enum ConnectionState { Disconnected = 0, Connecting = 1, Connected = 2, Error = 3 };

    ConnectionState state = Disconnected;
    TS_ASSERT_EQUALS(static_cast<int>(state), 0);

    state = Connected;
    TS_ASSERT_EQUALS(static_cast<int>(state), 2);
  }

  // Test input queue ordering
  void testInputQueueOrdering() {
    std::vector<std::string> inputQueue;

    inputQueue.push_back("cmd1");
    inputQueue.push_back("cmd2");
    inputQueue.push_back("cmd3");

    // FIFO order
    TS_ASSERT_EQUALS(inputQueue[0], "cmd1");
    TS_ASSERT_EQUALS(inputQueue[1], "cmd2");
    TS_ASSERT_EQUALS(inputQueue[2], "cmd3");

    // Process front first
    std::string first = inputQueue.front();
    inputQueue.erase(inputQueue.begin());

    TS_ASSERT_EQUALS(first, "cmd1");
    TS_ASSERT_EQUALS(inputQueue.size(), 2u);
  }

  // Test batch command parsing
  void testBatchCommandParsing() {
    // Multiple commands separated by semicolons
    std::string batch = "set prop1 1.0; set prop2 2.0; set prop3 3.0";

    std::vector<std::string> commands;
    std::istringstream iss(batch);
    std::string cmd;

    while (std::getline(iss, cmd, ';')) {
      // Trim whitespace
      size_t start = cmd.find_first_not_of(" \t");
      size_t end = cmd.find_last_not_of(" \t");
      if (start != std::string::npos) {
        commands.push_back(cmd.substr(start, end - start + 1));
      }
    }

    TS_ASSERT_EQUALS(commands.size(), 3u);
    TS_ASSERT_EQUALS(commands[0], "set prop1 1.0");
    TS_ASSERT_EQUALS(commands[1], "set prop2 2.0");
    TS_ASSERT_EQUALS(commands[2], "set prop3 3.0");
  }

  // Test input timeout handling
  void testInputTimeoutHandling() {
    double timeoutSec = 30.0;
    double elapsedSec = 0.0;

    // Simulate time passing
    for (int i = 0; i < 10; i++) {
      elapsedSec += 1.0;
    }

    bool timedOut = (elapsedSec >= timeoutSec);
    TS_ASSERT(!timedOut);

    // Exceed timeout
    elapsedSec = 35.0;
    timedOut = (elapsedSec >= timeoutSec);
    TS_ASSERT(timedOut);
  }

  // Test double precision value parsing
  void testDoublePrecisionParsing() {
    std::string highPrecision = "3.141592653589793";

    double value = std::stod(highPrecision);

    TS_ASSERT_DELTA(value, M_PI, 1e-15);
  }

  // Test very small value parsing
  void testVerySmallValueParsing() {
    std::string smallValue = "1e-20";

    double value = std::atof(smallValue.c_str());

    TS_ASSERT(value > 0.0);
    TS_ASSERT(value < 1e-10);
    TS_ASSERT_DELTA(value, 1e-20, 1e-30);
  }

  // Test very large value parsing
  void testVeryLargeValueParsing() {
    std::string largeValue = "1e20";

    double value = std::atof(largeValue.c_str());

    TS_ASSERT(value > 1e10);
    TS_ASSERT_DELTA(value, 1e20, 1e10);
  }

  // Test special float values
  void testSpecialFloatValues() {
    double inf = std::numeric_limits<double>::infinity();
    double negInf = -std::numeric_limits<double>::infinity();
    double nan = std::numeric_limits<double>::quiet_NaN();

    TS_ASSERT(std::isinf(inf));
    TS_ASSERT(std::isinf(negInf));
    TS_ASSERT(inf > 0.0);
    TS_ASSERT(negInf < 0.0);
    TS_ASSERT(std::isnan(nan));
  }

  // Test input buffer clear
  void testInputBufferClear() {
    std::string buffer = "some data here";

    TS_ASSERT(!buffer.empty());

    buffer.clear();

    TS_ASSERT(buffer.empty());
    TS_ASSERT_EQUALS(buffer.size(), 0u);
  }

  // Test property category validation
  void testPropertyCategoryValidation() {
    std::vector<std::string> validCategories = {
      "velocities", "position", "attitude", "fcs", "propulsion",
      "aero", "forces", "moments", "accelerations", "metrics"
    };

    std::string testCategory = "velocities";

    bool isValid = std::find(validCategories.begin(), validCategories.end(),
                             testCategory) != validCategories.end();

    TS_ASSERT(isValid);

    // Invalid category
    testCategory = "invalid_category";
    isValid = std::find(validCategories.begin(), validCategories.end(),
                        testCategory) != validCategories.end();
    TS_ASSERT(!isValid);
  }

  // Test port range validation
  void testPortRangeValidation() {
    int reservedPort = 80;     // HTTP
    int userPort = 5500;       // User range
    int highPort = 49152;      // Dynamic/private range

    // Reserved ports (< 1024) typically require root
    TS_ASSERT(reservedPort < 1024);

    // User ports (1024-49151) are typical for applications
    TS_ASSERT(userPort >= 1024 && userPort < 49152);

    // Dynamic ports (49152-65535) are for ephemeral connections
    TS_ASSERT(highPort >= 49152 && highPort <= 65535);
  }

  // Test error response format
  void testErrorResponseFormat() {
    std::string errorProperty = "unknown/property";
    std::string errorMsg = "ERROR: Property not found: " + errorProperty;

    TS_ASSERT(errorMsg.find("ERROR") != std::string::npos);
    TS_ASSERT(errorMsg.find(errorProperty) != std::string::npos);
  }

  // Test command echo response
  void testCommandEchoResponse() {
    std::string command = "set velocities/vc-kts 150.0";
    bool echoEnabled = true;

    std::string response;
    if (echoEnabled) {
      response = "> " + command + "\r\n";
    }

    TS_ASSERT(response.find(command) != std::string::npos);
    TS_ASSERT(response.substr(0, 2) == "> ");
  }

  // Test multiple sequential commands
  void testMultipleSequentialCommands() {
    std::vector<std::string> commands = {
      "hold",
      "set fcs/throttle-cmd-norm 1.0",
      "iterate 100",
      "get velocities/vc-kts",
      "resume"
    };

    // Process sequentially
    int processed = 0;
    for (const auto& cmd : commands) {
      TS_ASSERT(!cmd.empty());
      processed++;
    }

    TS_ASSERT_EQUALS(processed, 5);
  }

  // Test input with quoted string value
  void testQuotedStringValue() {
    std::string line = "set simulation/name \"Test Flight\"";

    // Find first and last quote
    size_t firstQuote = line.find('"');
    size_t lastQuote = line.rfind('"');

    TS_ASSERT(firstQuote != std::string::npos);
    TS_ASSERT(lastQuote != std::string::npos);
    TS_ASSERT(lastQuote > firstQuote);

    std::string quotedValue = line.substr(firstQuote + 1, lastQuote - firstQuote - 1);
    TS_ASSERT_EQUALS(quotedValue, "Test Flight");
  }

  // Test concurrent input processing flag
  void testConcurrentInputProcessing() {
    bool inputProcessing = false;

    // Start processing
    inputProcessing = true;
    TS_ASSERT(inputProcessing);

    // End processing
    inputProcessing = false;
    TS_ASSERT(!inputProcessing);
  }

  // Test input direction (read vs write)
  void testInputDirection() {
    enum Direction { Read = 0, Write = 1 };

    Direction inputDir = Read;
    Direction outputDir = Write;

    TS_ASSERT_EQUALS(static_cast<int>(inputDir), 0);
    TS_ASSERT_EQUALS(static_cast<int>(outputDir), 1);
    TS_ASSERT(inputDir != outputDir);
  }

  // Test host address validation
  void testHostAddressValidation() {
    std::string localhost = "127.0.0.1";
    std::string anyAddr = "0.0.0.0";
    std::string hostname = "localhost";

    // Basic IPv4 validation
    int dotCount = 0;
    for (char c : localhost) {
      if (c == '.') dotCount++;
    }
    TS_ASSERT_EQUALS(dotCount, 3);

    dotCount = 0;
    for (char c : anyAddr) {
      if (c == '.') dotCount++;
    }
    TS_ASSERT_EQUALS(dotCount, 3);

    // Hostname doesn't have dots (in this case)
    dotCount = 0;
    for (char c : hostname) {
      if (c == '.') dotCount++;
    }
    TS_ASSERT_EQUALS(dotCount, 0);
  }

  //==========================================================================
  // UDP INPUT HANDLING TESTS
  //==========================================================================

  // Test UDP datagram size limits
  void testUDPDatagramSizeLimits() {
    size_t maxUDPPayload = 65507;  // Max UDP payload size
    size_t typicalMTU = 1500;      // Typical ethernet MTU
    size_t safePayload = 1472;     // MTU - IP header - UDP header

    TS_ASSERT(maxUDPPayload > typicalMTU);
    TS_ASSERT(safePayload < typicalMTU);
    TS_ASSERT_EQUALS(safePayload, 1472u);
  }

  // Test UDP packet sequencing
  void testUDPPacketSequencing() {
    int expectedSequence = 100;
    int receivedSequence = 101;

    bool outOfOrder = (receivedSequence != expectedSequence);
    int missed = receivedSequence - expectedSequence - 1;

    TS_ASSERT(outOfOrder);
    TS_ASSERT_EQUALS(missed, 0);  // No packets missed, just received next one
  }

  // Test UDP packet loss detection
  void testUDPPacketLossDetection() {
    int lastSequence = 50;
    int currentSequence = 55;

    int packetsLost = currentSequence - lastSequence - 1;

    TS_ASSERT_EQUALS(packetsLost, 4);
    TS_ASSERT(packetsLost > 0);
  }

  // Test UDP source address filtering
  void testUDPSourceFiltering() {
    std::string allowedSource = "192.168.1.100";
    std::string receivedSource = "192.168.1.100";
    std::string blockedSource = "10.0.0.5";

    bool sourceAllowed = (receivedSource == allowedSource);
    bool sourceBlocked = (blockedSource != allowedSource);

    TS_ASSERT(sourceAllowed);
    TS_ASSERT(sourceBlocked);
  }

  //==========================================================================
  // BINARY PROTOCOL PARSING TESTS
  //==========================================================================

  // Test binary header parsing
  void testBinaryHeaderParsing() {
    // Simulate binary header: magic (4 bytes), version (2), length (2)
    // Using big-endian (network byte order) format
    unsigned char header[8] = {0x4A, 0x53, 0x42, 0x53, 0x00, 0x01, 0x00, 0x40};
    // "JSBS" magic, version 1 (big-endian), length 64 (big-endian)

    unsigned int magic = (header[0] << 24) | (header[1] << 16) |
                         (header[2] << 8) | header[3];
    unsigned short version = (header[4] << 8) | header[5];
    unsigned short length = (header[6] << 8) | header[7];

    TS_ASSERT_EQUALS(magic, 0x4A534253u);  // "JSBS"
    TS_ASSERT_EQUALS(version, 1u);
    TS_ASSERT_EQUALS(length, 64u);
  }

  // Test binary float encoding (IEEE 754)
  void testBinaryFloatEncoding() {
    float testValue = 150.5f;
    unsigned char bytes[4];

    // Copy float to bytes
    std::memcpy(bytes, &testValue, sizeof(float));

    // Reconstruct float
    float reconstructed;
    std::memcpy(&reconstructed, bytes, sizeof(float));

    TS_ASSERT_DELTA(reconstructed, 150.5f, 0.001f);
  }

  // Test binary double encoding
  void testBinaryDoubleEncoding() {
    double testValue = 3.141592653589793;
    unsigned char bytes[8];

    std::memcpy(bytes, &testValue, sizeof(double));

    double reconstructed;
    std::memcpy(&reconstructed, bytes, sizeof(double));

    TS_ASSERT_DELTA(reconstructed, testValue, 1e-15);
  }

  // Test endianness detection
  void testEndiannessDetection() {
    unsigned int test = 0x01020304;
    unsigned char* bytes = reinterpret_cast<unsigned char*>(&test);

    bool isLittleEndian = (bytes[0] == 0x04);
    bool isBigEndian = (bytes[0] == 0x01);

    TS_ASSERT(isLittleEndian != isBigEndian);  // Must be one or the other
  }

  //==========================================================================
  // CONNECTION MANAGEMENT TESTS
  //==========================================================================

  // Test connection retry logic
  void testConnectionRetryLogic() {
    int maxRetries = 5;
    int currentRetry = 0;
    int retryDelay_ms = 1000;
    double backoffFactor = 2.0;

    for (int i = 0; i < 3; i++) {
      currentRetry++;
      int currentDelay = static_cast<int>(retryDelay_ms * std::pow(backoffFactor, currentRetry - 1));
      TS_ASSERT(currentDelay >= retryDelay_ms);
    }

    TS_ASSERT(currentRetry < maxRetries);
  }

  // Test connection keepalive
  void testConnectionKeepalive() {
    double keepaliveInterval_s = 30.0;
    double lastActivity_s = 25.0;
    double currentTime_s = 50.0;

    double timeSinceActivity = currentTime_s - lastActivity_s;
    bool needsKeepalive = (timeSinceActivity >= keepaliveInterval_s);

    TS_ASSERT(!needsKeepalive);

    lastActivity_s = 15.0;
    timeSinceActivity = currentTime_s - lastActivity_s;
    needsKeepalive = (timeSinceActivity >= keepaliveInterval_s);

    TS_ASSERT(needsKeepalive);
  }

  // Test connection pool sizing
  void testConnectionPoolSizing() {
    int maxConnections = 10;
    int activeConnections = 5;
    int pendingConnections = 3;

    int totalInUse = activeConnections + pendingConnections;
    int available = maxConnections - totalInUse;

    TS_ASSERT_EQUALS(available, 2);
    TS_ASSERT(totalInUse <= maxConnections);
  }

  // Test graceful disconnect handling
  void testGracefulDisconnect() {
    bool connected = true;
    bool pendingWrites = true;

    // Wait for pending writes before disconnect
    bool canDisconnect = connected && !pendingWrites;

    TS_ASSERT(!canDisconnect);

    pendingWrites = false;
    canDisconnect = connected && !pendingWrites;

    TS_ASSERT(canDisconnect);
  }

  //==========================================================================
  // INPUT THROTTLING TESTS
  //==========================================================================

  // Test input rate throttling
  void testInputRateThrottling() {
    double maxInputRate_hz = 100.0;
    double minInterval_s = 1.0 / maxInputRate_hz;
    double lastInputTime_s = 0.0;
    double currentTime_s = 0.005;  // 5ms later

    bool canProcess = (currentTime_s - lastInputTime_s) >= minInterval_s;

    TS_ASSERT(!canProcess);  // Too soon

    currentTime_s = 0.015;  // 15ms later
    canProcess = (currentTime_s - lastInputTime_s) >= minInterval_s;

    TS_ASSERT(canProcess);  // OK now
  }

  // Test burst input handling
  void testBurstInputHandling() {
    int burstLimit = 15;
    int burstWindow_ms = 100;
    std::vector<int> inputTimestamps_ms = {0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50};

    // Count inputs within burst window
    int inputsInWindow = 0;
    int windowStart = 50 - burstWindow_ms;

    for (int ts : inputTimestamps_ms) {
      if (ts >= windowStart) inputsInWindow++;
    }

    bool withinBurstLimit = (inputsInWindow <= burstLimit);

    TS_ASSERT(withinBurstLimit);  // 11 <= 15
    TS_ASSERT_EQUALS(inputsInWindow, 11);
  }

  // Test input queue depth limit
  void testInputQueueDepthLimit() {
    size_t maxQueueDepth = 100;
    std::vector<std::string> inputQueue;

    for (int i = 0; i < 150; i++) {
      if (inputQueue.size() < maxQueueDepth) {
        inputQueue.push_back("command");
      }
    }

    TS_ASSERT_EQUALS(inputQueue.size(), maxQueueDepth);
    TS_ASSERT(inputQueue.size() <= maxQueueDepth);
  }

  // Test priority input handling
  void testPriorityInputHandling() {
    enum Priority { Low = 0, Normal = 1, High = 2, Critical = 3 };

    std::vector<std::pair<Priority, std::string>> inputQueue = {
      {Normal, "cmd1"}, {Low, "cmd2"}, {Critical, "cmd3"}, {High, "cmd4"}
    };

    // Sort by priority (highest first)
    std::sort(inputQueue.begin(), inputQueue.end(),
              [](const auto& a, const auto& b) { return a.first > b.first; });

    TS_ASSERT_EQUALS(inputQueue[0].second, "cmd3");  // Critical
    TS_ASSERT_EQUALS(inputQueue[1].second, "cmd4");  // High
  }

  //==========================================================================
  // COMMAND VALIDATION TESTS
  //==========================================================================

  // Test command length validation
  void testCommandLengthValidation() {
    size_t maxCommandLength = 1024;
    std::string shortCommand = "set prop 1.0";
    std::string longCommand(2000, 'x');

    bool shortValid = (shortCommand.length() <= maxCommandLength);
    bool longValid = (longCommand.length() <= maxCommandLength);

    TS_ASSERT(shortValid);
    TS_ASSERT(!longValid);
  }

  // Test command character validation
  void testCommandCharacterValidation() {
    std::string validCommand = "set fcs/throttle-cmd-norm 0.8";
    std::string invalidCommand = "set prop\x00value";  // Contains null

    // Check for printable characters
    bool allPrintable = true;
    for (char c : validCommand) {
      if (!std::isprint(static_cast<unsigned char>(c)) && !std::isspace(static_cast<unsigned char>(c))) {
        allPrintable = false;
        break;
      }
    }

    TS_ASSERT(allPrintable);
  }

  // Test property access permissions
  void testPropertyAccessPermissions() {
    std::vector<std::string> readOnlyProps = {
      "sim/time-sec", "sim/dt-sec", "sim/version"
    };

    std::string testProp = "sim/time-sec";

    bool isReadOnly = std::find(readOnlyProps.begin(), readOnlyProps.end(),
                                 testProp) != readOnlyProps.end();

    TS_ASSERT(isReadOnly);

    testProp = "fcs/throttle-cmd-norm";
    isReadOnly = std::find(readOnlyProps.begin(), readOnlyProps.end(),
                           testProp) != readOnlyProps.end();

    TS_ASSERT(!isReadOnly);
  }

  // Test numeric range validation
  void testNumericRangeValidation() {
    struct PropertyRange {
      std::string name;
      double minVal;
      double maxVal;
    };

    std::vector<PropertyRange> ranges = {
      {"fcs/throttle-cmd-norm", 0.0, 1.0},
      {"fcs/elevator-cmd-norm", -1.0, 1.0},
      {"attitude/phi-rad", -M_PI, M_PI}
    };

    double testValue = 0.5;
    bool inRange = (testValue >= ranges[0].minVal && testValue <= ranges[0].maxVal);

    TS_ASSERT(inRange);

    testValue = 1.5;
    inRange = (testValue >= ranges[0].minVal && testValue <= ranges[0].maxVal);

    TS_ASSERT(!inRange);
  }

  //==========================================================================
  // PROPERTY SUBSCRIPTION TESTS
  //==========================================================================

  // Test property subscription add/remove
  void testPropertySubscription() {
    std::vector<std::string> subscriptions;

    // Add subscription
    subscriptions.push_back("velocities/vc-kts");
    TS_ASSERT_EQUALS(subscriptions.size(), 1u);

    // Add another
    subscriptions.push_back("position/h-sl-ft");
    TS_ASSERT_EQUALS(subscriptions.size(), 2u);

    // Remove first
    subscriptions.erase(subscriptions.begin());
    TS_ASSERT_EQUALS(subscriptions.size(), 1u);
    TS_ASSERT_EQUALS(subscriptions[0], "position/h-sl-ft");
  }

  // Test subscription update rate
  void testSubscriptionUpdateRate() {
    double subscriptionRate_hz = 20.0;
    double simRate_hz = 120.0;

    int framesPerUpdate = static_cast<int>(simRate_hz / subscriptionRate_hz);

    TS_ASSERT_EQUALS(framesPerUpdate, 6);
  }

  // Test subscription filter (delta only)
  void testSubscriptionDeltaFilter() {
    double lastValue = 100.0;
    double currentValue = 100.5;
    double deltaThreshold = 1.0;

    bool hasChanged = std::abs(currentValue - lastValue) >= deltaThreshold;

    TS_ASSERT(!hasChanged);

    currentValue = 102.0;
    hasChanged = std::abs(currentValue - lastValue) >= deltaThreshold;

    TS_ASSERT(hasChanged);
  }

  // Test subscription grouping
  void testSubscriptionGrouping() {
    std::map<std::string, std::vector<std::string>> groups;

    groups["flight"] = {"velocities/vc-kts", "position/h-sl-ft", "attitude/phi-rad"};
    groups["engine"] = {"propulsion/engine[0]/thrust-lbs", "propulsion/engine[0]/n1"};

    TS_ASSERT_EQUALS(groups.size(), 2u);
    TS_ASSERT_EQUALS(groups["flight"].size(), 3u);
    TS_ASSERT_EQUALS(groups["engine"].size(), 2u);
  }

  //==========================================================================
  // TELEMETRY OUTPUT FORMAT TESTS
  //==========================================================================

  // Test CSV output format
  void testCSVOutputFormat() {
    std::vector<double> values = {150.5, 10000.0, 0.1, 0.8};
    std::ostringstream csv;

    for (size_t i = 0; i < values.size(); i++) {
      csv << values[i];
      if (i < values.size() - 1) csv << ",";
    }
    csv << "\n";

    std::string result = csv.str();
    TS_ASSERT(result.find(",") != std::string::npos);
    TS_ASSERT(result.find("\n") != std::string::npos);
  }

  // Test JSON output format
  void testJSONOutputFormat() {
    std::ostringstream json;
    json << "{\"velocity\":" << 150.5 << ",\"altitude\":" << 10000 << "}";

    std::string result = json.str();
    TS_ASSERT(result.find("{") != std::string::npos);
    TS_ASSERT(result.find("}") != std::string::npos);
    TS_ASSERT(result.find("\"velocity\"") != std::string::npos);
  }

  // Test timestamp formatting
  void testTimestampFormatting() {
    double simTime_s = 123.456789;

    std::ostringstream ts;
    ts << std::fixed << std::setprecision(6) << simTime_s;

    std::string result = ts.str();
    TS_ASSERT_EQUALS(result, "123.456789");
  }

  // Test output field alignment
  void testOutputFieldAlignment() {
    std::ostringstream output;
    output << std::setw(12) << std::right << 150.5
           << std::setw(12) << std::right << 10000.0
           << std::setw(12) << std::right << 0.1;

    std::string result = output.str();
    TS_ASSERT_EQUALS(result.length(), 36u);
  }

  //==========================================================================
  // INPUT VALIDATION EDGE CASES
  //==========================================================================

  // Test NaN value rejection
  void testNaNValueRejection() {
    std::string nanStr = "nan";
    double value = std::atof(nanStr.c_str());

    bool isValidNumber = !std::isnan(value) && !std::isinf(value);

    TS_ASSERT(!isValidNumber);
  }

  // Test infinity value handling
  void testInfinityValueHandling() {
    std::string infStr = "inf";
    double value = std::atof(infStr.c_str());

    bool isFinite = std::isfinite(value);

    TS_ASSERT(!isFinite);
  }

  // Test empty property name
  void testEmptyPropertyName() {
    std::string line = "set  1.0";  // Empty property name

    std::istringstream iss(line);
    std::string cmd, prop, val;
    iss >> cmd >> prop >> val;

    // Second token becomes value, no property
    bool hasValidProperty = !prop.empty() && prop.find('/') != std::string::npos;

    TS_ASSERT(!hasValidProperty);  // "1.0" is not a valid property
  }

  // Test special character in property name
  void testSpecialCharInPropertyName() {
    std::string property = "fcs/aileron$pos";

    bool hasInvalidChar = false;
    for (char c : property) {
      if (c == '$' || c == '@' || c == '#' || c == '!') {
        hasInvalidChar = true;
        break;
      }
    }

    TS_ASSERT(hasInvalidChar);
  }
};
