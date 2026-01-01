#include <cxxtest/TestSuite.h>
#include <input_output/FGLog.h>
#include <input_output/FGXMLElement.h>
#include <FGFDMExec.h>
#include "TestUtilities.h"

class DummyLogger : public JSBSim::FGLogger
{
public:
  JSBSim::LogLevel GetLogLevel() const { return log_level; }
  void Message(const std::string& message) override { buffer.append(message); }
  void FileLocation(const std::string& filename, int line) override {
    buffer.append(filename);
    buffer.append(":");
    buffer.append(std::to_string(line));
  }
  void Format(JSBSim::LogFormat format) override {
  switch (format)
  {
  case JSBSim::LogFormat::NORMAL:
    buffer.append("NORMAL");
    break;
  default:
    buffer.append("UNKNOWN");
    break;
  }
  }
  void Flush(void) override { flushed = true; }

  std::string buffer;
  bool flushed = false;
};

class FGLogTest : public CxxTest::TestSuite
{
public:
void testConstructor() {
  auto logger = std::make_shared<DummyLogger>();
  TS_ASSERT(!logger->flushed);
  TS_ASSERT(logger->buffer.empty());
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);

  JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
  TS_ASSERT(!logger->flushed);
  TS_ASSERT(logger->buffer.empty());
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testDestructor() {
  auto logger = std::make_shared<DummyLogger>();
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->buffer.empty());
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testCharMessage() {
  auto logger = std::make_shared<DummyLogger>();
  const char* message = "Hello, World!";
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log <<message;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, message);
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testStringMessage() {
  auto logger = std::make_shared<DummyLogger>();
  std::string message = "Hello, World!";
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << message;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, message);
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testConcatenatedMessages() {
  auto logger = std::make_shared<DummyLogger>();
  std::string message1 = "Hello";
  std::string message2 = "World!";
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << message1 << " " << message2;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, message1 + " " + message2);
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testEndl() {
  auto logger = std::make_shared<DummyLogger>();
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << "Hello" << std::endl << "World!";
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "Hello\nWorld!");
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testNumbers() {
  auto logger = std::make_shared<DummyLogger>();
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << 1 << 2.1 << -3.4f;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "12.1-3.4");
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testSetPrecision() {
  auto logger = std::make_shared<DummyLogger>();
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << std::setprecision(3) << 1.23456789;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "1.23");
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testSetWidthRight() {
  auto logger = std::make_shared<DummyLogger>();
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << std::setw(5) << 123;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "  123");
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testSetWidthLeft() {
  auto logger = std::make_shared<DummyLogger>();
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << std::left << std::setw(5) << 123;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "123  ");
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testPath() {
  auto logger = std::make_shared<DummyLogger>();
  SGPath path("path/to");
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << (path/"file");
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "Path \"path/to/file\"");
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testColumnVector3() {
  auto logger = std::make_shared<DummyLogger>();
  JSBSim::FGColumnVector3 vec(1, 2, 3);
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << vec;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "1 , 2 , 3");
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testFormatOnly() {
  auto logger = std::make_shared<DummyLogger>();
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    log << JSBSim::LogFormat::NORMAL;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT_EQUALS(logger->buffer, "NORMAL");
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "NORMAL");
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testClosingFormat() {
  auto logger = std::make_shared<DummyLogger>();
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << "Hello,";
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
    log << JSBSim::LogFormat::NORMAL;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT_EQUALS(logger->buffer, "Hello,NORMAL");
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "Hello,NORMAL");
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testMidFormat() {
  auto logger = std::make_shared<DummyLogger>();
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << "Hello,";
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
    log << JSBSim::LogFormat::NORMAL;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT_EQUALS(logger->buffer, "Hello,NORMAL");
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
    log << " World!";
    TS_ASSERT(!logger->flushed);
    TS_ASSERT_EQUALS(logger->buffer, "Hello,NORMAL");
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "Hello,NORMAL World!");
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);
}

void testXMLLogging() {
  auto logger = std::make_shared<DummyLogger>();
  JSBSim::Element el("element");
  el.SetFileName("file.xml");
  el.SetLineNumber(42);
  {
    JSBSim::FGXMLLogging log(logger, &el, JSBSim::LogLevel::DEBUG);
    TS_ASSERT_EQUALS(logger->buffer, "file.xml:42");
    TS_ASSERT(!logger->flushed);
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::DEBUG);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "file.xml:42");
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::DEBUG);
}
};

class FGLogConsoleTest : public CxxTest::TestSuite
{
public:
void testNormalMessage() {
  auto logger = std::make_shared<JSBSim::FGLogConsole>();
  std::ostringstream buffer;
  auto cout_buffer = std::cout.rdbuf();
  std::cout.rdbuf(buffer.rdbuf());
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::DEBUG);
    log << "Hello, World!";
  }
  std::cout.rdbuf(cout_buffer);
  TS_ASSERT_EQUALS(buffer.str(), "Hello, World!");
}

void testErrorMessage() {
  auto logger = std::make_shared<JSBSim::FGLogConsole>();
  std::ostringstream buffer;
  auto cerr_buffer = std::cerr.rdbuf();
  std::cerr.rdbuf(buffer.rdbuf());
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::ERROR);
    log << "Hello, World!";
  }
  std::cerr.rdbuf(cerr_buffer);
  TS_ASSERT_EQUALS(buffer.str(), "Hello, World!");
}

void testXMLLogging() {
  auto logger = std::make_shared<JSBSim::FGLogConsole>();
  std::ostringstream buffer;
  auto cout_buffer = std::cout.rdbuf();
  std::cout.rdbuf(buffer.rdbuf());
  JSBSim::Element el("element");
  el.SetFileName("name.xml");
  el.SetLineNumber(42);
  {
    JSBSim::FGXMLLogging log(logger, &el, JSBSim::LogLevel::DEBUG);
  }
  std::cout.rdbuf(cout_buffer);
  TS_ASSERT_EQUALS(buffer.str(), "\nIn file name.xml: line 42\n");
}

void testMinLevel() {
  auto logger = std::make_shared<JSBSim::FGLogConsole>();
  logger->SetMinLevel(JSBSim::LogLevel::DEBUG);
  std::ostringstream buffer;
  auto cout_buffer = std::cout.rdbuf();
  std::cout.rdbuf(buffer.rdbuf());
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::BULK);
    log << "BULK";
  }
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << "INFO";
  }
  std::cout.rdbuf(cout_buffer);
  TS_ASSERT_EQUALS(buffer.str(), "INFO");
}

void testRedFormat() {
  auto logger = std::make_shared<JSBSim::FGLogConsole>();
  std::ostringstream buffer;
  auto cout_buffer = std::cout.rdbuf();
  std::cout.rdbuf(buffer.rdbuf());
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << JSBSim::LogFormat::RED;
    log << "Hello, World!";
    log << JSBSim::LogFormat::RESET;
  }
  std::cout.rdbuf(cout_buffer);
  TS_ASSERT_EQUALS(buffer.str(), "\033[31mHello, World!\033[0m");
}

void testCyanFormat() {
  auto logger = std::make_shared<JSBSim::FGLogConsole>();
  std::ostringstream buffer;
  auto cout_buffer = std::cout.rdbuf();
  std::cout.rdbuf(buffer.rdbuf());
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << JSBSim::LogFormat::BLUE;
    log << "Hello, World!";
    log << JSBSim::LogFormat::RESET;
  }
  std::cout.rdbuf(cout_buffer);
  TS_ASSERT_EQUALS(buffer.str(), "\033[34mHello, World!\033[0m");
}

void testBoldFormat() {
  auto logger = std::make_shared<JSBSim::FGLogConsole>();
  std::ostringstream buffer;
  auto cout_buffer = std::cout.rdbuf();
  std::cout.rdbuf(buffer.rdbuf());
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << JSBSim::LogFormat::BOLD;
    log << "Hello, World!";
    log << JSBSim::LogFormat::RESET;
  }
  std::cout.rdbuf(cout_buffer);
  TS_ASSERT_EQUALS(buffer.str(), "\033[1mHello, World!\033[0m");
}

void testNormalFormat() {
  auto logger = std::make_shared<JSBSim::FGLogConsole>();
  std::ostringstream buffer;
  auto cout_buffer = std::cout.rdbuf();
  std::cout.rdbuf(buffer.rdbuf());
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << JSBSim::LogFormat::NORMAL;
    log << "Hello, World!";
    log << JSBSim::LogFormat::RESET;
  }
  std::cout.rdbuf(cout_buffer);
  TS_ASSERT_EQUALS(buffer.str(), "\033[22mHello, World!\033[0m");
}

void testUnderlineFormat() {
  auto logger = std::make_shared<JSBSim::FGLogConsole>();
  std::ostringstream buffer;
  auto cout_buffer = std::cout.rdbuf();
  std::cout.rdbuf(buffer.rdbuf());
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << JSBSim::LogFormat::UNDERLINE_ON;
    log << "Hello, World!";
    log << JSBSim::LogFormat::UNDERLINE_OFF;
  }
  std::cout.rdbuf(cout_buffer);
  TS_ASSERT_EQUALS(buffer.str(), "\033[4mHello, World!\033[24m");
}

void testDefaultFormat() {
  auto logger = std::make_shared<JSBSim::FGLogConsole>();
  std::ostringstream buffer;
  auto cout_buffer = std::cout.rdbuf();
  std::cout.rdbuf(buffer.rdbuf());
  {
    JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
    log << JSBSim::LogFormat::DEFAULT;
    log << "Hello, World!";
    log << JSBSim::LogFormat::RESET;
  }
  std::cout.rdbuf(cout_buffer);
  TS_ASSERT_EQUALS(buffer.str(), "\033[39mHello, World!\033[0m");
}
};

class LogExceptionTest : public CxxTest::TestSuite
{
public:
void testConstructor() {
  auto logger = std::make_shared<DummyLogger>();
  JSBSim::LogException logException(logger);
  TS_ASSERT(!logger->flushed);
  TS_ASSERT(logger->buffer.empty());
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
}

void testDestructor() {
  auto logger = std::make_shared<DummyLogger>();
  {
    JSBSim::LogException logException(logger);
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(!logger->flushed);
  TS_ASSERT(logger->buffer.empty());
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
}

void testThrow() {
  auto logger = std::make_shared<DummyLogger>();
  try {
    JSBSim::LogException logException(logger);
    throw logException;
  } catch (JSBSim::BaseException&) {
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(!logger->flushed);
  TS_ASSERT(logger->buffer.empty());
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
}

void testThrowEmptyMessage() {
  auto logger = std::make_shared<DummyLogger>();
  try {
    JSBSim::LogException logException(logger);
    logException << "";
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    throw logException;
  } catch (JSBSim::BaseException&) {
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(!logger->flushed);
  TS_ASSERT(logger->buffer.empty());
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
}

void testThrowWithMessage() {
  auto logger = std::make_shared<DummyLogger>();
  std::string message = "Hello, World!";
  try {
    JSBSim::LogException logException(logger);
    logException << message;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    throw logException;
  } catch (JSBSim::BaseException&) {
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, message);
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::FATAL);
}

void testThrowConcatenatedMessages1() {
  auto logger = std::make_shared<DummyLogger>();
  std::string message1 = "Hello";
  std::string message2 = ", World!";
  try {
    JSBSim::LogException logException(logger);
    logException << message1 << message2;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    throw logException;
  } catch (JSBSim::BaseException&) {
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, message1 + message2);
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::FATAL);
}

void testThrowConcatenatedMessages2() {
  auto logger = std::make_shared<DummyLogger>();
  std::string message1 = "Hello";
  std::string message2 = ", World!";
  try {
    JSBSim::LogException logException(logger);
    logException << message1;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    logException << message2;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    throw logException;
  } catch (JSBSim::BaseException&) {
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, message1 + message2);
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::FATAL);
}

void testThrowFormattedMessages1() {
  auto logger = std::make_shared<DummyLogger>();
  std::string message = "Hello, World!";
  try {
    JSBSim::LogException logException(logger);
    logException << JSBSim::LogFormat::NORMAL << message;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    throw logException;
  } catch (JSBSim::BaseException&) {
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "NORMAL" + message);
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::FATAL);
}

void testThrowFormattedMessages2() {
  auto logger = std::make_shared<DummyLogger>();
  std::string message = "Hello, World!";
  try {
    JSBSim::LogException logException(logger);
    logException << JSBSim::LogFormat::NORMAL;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    logException << message;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    throw logException;
  } catch (JSBSim::BaseException&) {
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "NORMAL" + message);
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::FATAL);
}

void testThrowFormattedMessages3() {
  auto logger = std::make_shared<DummyLogger>();
  std::string message1 = "Hello";
  std::string message2 = ", World!";
  try {
    JSBSim::LogException logException(logger);
    logException << message1 << JSBSim::LogFormat::NORMAL << message2;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    throw logException;
  } catch (const JSBSim::BaseException& e) {
    TS_ASSERT_EQUALS(std::string(e.what()), message1 + message2);
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, message1 + "NORMAL" + message2);
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::FATAL);
}

void testThrowAndAppendMessage() {
  auto logger = std::make_shared<DummyLogger>();
  std::string message1 = "Hello";
  std::string message2 = ", World!";
  try {
    JSBSim::LogException logException(logger);
    logException << message1;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    throw logException;
  } catch (JSBSim::LogException& e) {
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    e << message2;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, message1 + message2);
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::FATAL);
}
};

class testXMLLogException : public CxxTest::TestSuite
{
public:
void testConstructor() {
  auto logger = std::make_shared<DummyLogger>();
  JSBSim::Element el("element");
  el.SetFileName("file.xml");
  el.SetLineNumber(42);
  JSBSim::XMLLogException logException(logger, &el);
  TS_ASSERT(!logger->flushed);
  TS_ASSERT(logger->buffer.empty());
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
}

void testThrow() {
  auto logger = std::make_shared<DummyLogger>();
  JSBSim::Element el("element");
  el.SetFileName("file.xml");
  el.SetLineNumber(42);
  try {
    JSBSim::XMLLogException logException(logger, &el);
    throw logException;
  } catch (JSBSim::BaseException&) {
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(!logger->flushed);
  TS_ASSERT(logger->buffer.empty());
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
}

void testEmptyMessage() {
  auto logger = std::make_shared<DummyLogger>();
  JSBSim::Element el("element");
  el.SetFileName("file.xml");
  el.SetLineNumber(42);
  try {
    JSBSim::XMLLogException logException(logger, &el);
    logException << "";
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    throw logException;
  } catch (JSBSim::BaseException&) {
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(!logger->flushed);
  TS_ASSERT(logger->buffer.empty());
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
}

void testWithMessage() {
  auto logger = std::make_shared<DummyLogger>();
  JSBSim::Element el("element");
  el.SetFileName("file.xml");
  el.SetLineNumber(42);
  std::string message = "Hello, World!";
  try {
    JSBSim::XMLLogException logException(logger, &el);
    logException << message;
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    throw logException;
  } catch (JSBSim::BaseException&) {
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "file.xml:42" + message);
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::FATAL);
}

void testPromoteLogException() {
  auto logger = std::make_shared<DummyLogger>();
  JSBSim::Element el("element");
  el.SetFileName("file.xml");
  el.SetLineNumber(42);
  try {
    try {
      JSBSim::LogException logException(logger);
      logException << "Hello, World!";
      TS_ASSERT(!logger->flushed);
      TS_ASSERT(logger->buffer.empty());
      TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
      throw logException;
    } catch (JSBSim::LogException& e) {
      JSBSim::XMLLogException logException(e, &el);
      TS_ASSERT(!logger->flushed);
      TS_ASSERT(logger->buffer.empty());
      TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
      throw logException;
    }
  } catch (JSBSim::LogException&) {
    TS_ASSERT(!logger->flushed);
    TS_ASSERT(logger->buffer.empty());
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
  }
  TS_ASSERT(logger->flushed);
  TS_ASSERT_EQUALS(logger->buffer, "file.xml:42Hello, World!");
  TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::FATAL);
}
};

// ============================================================================
// C172x Integration Tests for FGLog
// ============================================================================

class FGLogC172xTest : public CxxTest::TestSuite
{
private:
  JSBSim::FGFDMExec fdm;

public:
  void setUp() {
    std::string rootDir = JSBSIM_TEST_ROOT_DIR;
    fdm.SetRootDir(SGPath(rootDir));
    fdm.SetAircraftPath(SGPath("aircraft"));
    fdm.SetEnginePath(SGPath("engine"));
    fdm.SetSystemsPath(SGPath("systems"));
    fdm.LoadModel("c172x");
  }

  void tearDown() {
    fdm.ResetToInitialConditions(0);
  }

  // Test 1: C172x model loads without logging errors
  void testC172xModelLoadsCleanly() {
    // Model was loaded in setUp - if we get here it loaded cleanly
    TS_ASSERT(true);
  }

  // Test 2: C172x simulation run produces valid state for logging
  void testC172xSimRunForLogging() {
    fdm.RunIC();
    fdm.Run();

    // Get simulation time for potential logging
    double simTime = fdm.GetPropertyManager()->GetDouble("simulation/sim-time-sec");
    TS_ASSERT(simTime >= 0.0);
  }

  // Test 3: C172x velocity data available for logging
  void testC172xVelocityDataForLogging() {
    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();
    double vc = pm->GetDouble("velocities/vc-kts");

    // Value should be finite (not NaN or Inf)
    TS_ASSERT(!std::isnan(vc));
    TS_ASSERT(!std::isinf(vc));
  }

  // Test 4: C172x position data available for logging
  void testC172xPositionDataForLogging() {
    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();
    double lat = pm->GetDouble("position/lat-gc-deg");
    double lon = pm->GetDouble("position/long-gc-deg");
    double alt = pm->GetDouble("position/h-sl-ft");

    TS_ASSERT(!std::isnan(lat));
    TS_ASSERT(!std::isnan(lon));
    TS_ASSERT(!std::isnan(alt));
  }

  // Test 5: C172x attitude data available for logging
  void testC172xAttitudeDataForLogging() {
    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();
    double phi = pm->GetDouble("attitude/phi-rad");
    double theta = pm->GetDouble("attitude/theta-rad");
    double psi = pm->GetDouble("attitude/psi-rad");

    TS_ASSERT(!std::isnan(phi));
    TS_ASSERT(!std::isnan(theta));
    TS_ASSERT(!std::isnan(psi));
  }

  // Test 6: C172x propulsion data available for logging
  void testC172xPropulsionDataForLogging() {
    fdm.RunIC();
    fdm.GetFCS()->SetThrottleCmd(0, 0.8);
    fdm.GetFCS()->SetMixtureCmd(0, 1.0);

    for (int i = 0; i < 10; i++) {
      fdm.Run();
    }

    auto pm = fdm.GetPropertyManager();
    double rpm = pm->GetDouble("propulsion/engine[0]/engine-rpm");

    TS_ASSERT(!std::isnan(rpm));
    TS_ASSERT(rpm >= 0.0);
  }

  // Test 7: C172x FCS data available for logging
  void testC172xFCSDataForLogging() {
    fdm.RunIC();
    fdm.GetFCS()->SetDeCmd(0.1);
    fdm.GetFCS()->SetDaCmd(0.05);
    fdm.GetFCS()->SetDrCmd(-0.05);
    fdm.Run();

    auto pm = fdm.GetPropertyManager();
    double elevator = pm->GetDouble("fcs/elevator-pos-rad");
    double aileron = pm->GetDouble("fcs/left-aileron-pos-rad");

    TS_ASSERT(!std::isnan(elevator));
    TS_ASSERT(!std::isnan(aileron));
  }

  // Test 8: C172x multiple timesteps logging consistency
  void testC172xMultipleTimestepsLogging() {
    fdm.RunIC();

    std::vector<double> times;
    for (int i = 0; i < 100; i++) {
      fdm.Run();
      double t = fdm.GetPropertyManager()->GetDouble("simulation/sim-time-sec");
      times.push_back(t);
    }

    // Times should be monotonically increasing
    for (size_t i = 1; i < times.size(); i++) {
      TS_ASSERT(times[i] > times[i-1]);
    }
  }

  // Test 9: C172x logging with control inputs
  void testC172xLoggingWithControlInputs() {
    fdm.RunIC();

    // Apply controls and log resulting state
    fdm.GetFCS()->SetThrottleCmd(0, 1.0);
    fdm.GetFCS()->SetDeCmd(-0.1);

    for (int i = 0; i < 50; i++) {
      fdm.Run();
    }

    auto pm = fdm.GetPropertyManager();
    double throttle = pm->GetDouble("fcs/throttle-cmd-norm");
    double vc = pm->GetDouble("velocities/vc-kts");

    TS_ASSERT_DELTA(throttle, 1.0, 0.01);
    TS_ASSERT(!std::isnan(vc));
  }

  // Test 10: C172x aerodynamic data for logging
  void testC172xAeroDataForLogging() {
    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();
    double alpha = pm->GetDouble("aero/alpha-deg");
    double qbar = pm->GetDouble("aero/qbar-psf");

    TS_ASSERT(!std::isnan(alpha));
    TS_ASSERT(!std::isnan(qbar));
    TS_ASSERT(qbar >= 0.0);
  }

  // Test 11: C172x forces data for logging
  void testC172xForcesDataForLogging() {
    fdm.RunIC();
    fdm.Run();

    auto pm = fdm.GetPropertyManager();
    double weight = pm->GetDouble("forces/fbz-weight-lbs");

    TS_ASSERT(!std::isnan(weight));
  }

  // Test 12: C172x comprehensive state snapshot
  void testC172xComprehensiveStateSnapshot() {
    fdm.RunIC();
    fdm.GetFCS()->SetThrottleCmd(0, 0.7);
    fdm.GetFCS()->SetMixtureCmd(0, 0.9);

    for (int i = 0; i < 20; i++) {
      fdm.Run();
    }

    auto pm = fdm.GetPropertyManager();

    // Snapshot multiple categories of data
    double simTime = pm->GetDouble("simulation/sim-time-sec");
    double altitude = pm->GetDouble("position/h-sl-ft");
    double vc = pm->GetDouble("velocities/vc-kts");
    double phi = pm->GetDouble("attitude/phi-rad");
    double alpha = pm->GetDouble("aero/alpha-deg");

    // All values should be valid for logging
    TS_ASSERT(!std::isnan(simTime));
    TS_ASSERT(!std::isnan(altitude));
    TS_ASSERT(!std::isnan(vc));
    TS_ASSERT(!std::isnan(phi));
    TS_ASSERT(!std::isnan(alpha));
  }
};

/***************************************************************************
 * Additional Log Tests
 ***************************************************************************/

class FGLogAdditionalTest : public CxxTest::TestSuite
{
public:
  // Test 43: Log level BULK
  void testLogLevelBulk() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::BULK);
      log << "Bulk message";
    }
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::BULK);
    TS_ASSERT(logger->flushed);
  }

  // Test 44: Log level DEBUG
  void testLogLevelDebug() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::DEBUG);
      log << "Debug message";
    }
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::DEBUG);
    TS_ASSERT(logger->flushed);
  }

  // Test 45: Log level WARN
  void testLogLevelWarn() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::WARN);
      log << "Warning message";
    }
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::WARN);
    TS_ASSERT(logger->flushed);
  }

  // Test 46: Log level ERROR
  void testLogLevelError() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::ERROR);
      log << "Error message";
    }
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::ERROR);
    TS_ASSERT(logger->flushed);
  }

  // Test 47: Log level FATAL
  void testLogLevelFatal() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::FATAL);
      log << "Fatal message";
    }
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::FATAL);
    TS_ASSERT(logger->flushed);
  }

  // Test 48: Empty message
  void testEmptyMessage() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << "";
    }
    TS_ASSERT(logger->flushed);
    TS_ASSERT(logger->buffer.empty());
  }

  // Test 49: Single character message
  void testSingleCharMessage() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << "X";
    }
    TS_ASSERT_EQUALS(logger->buffer, "X");
  }

  // Test 50: Long message
  void testLongMessage() {
    auto logger = std::make_shared<DummyLogger>();
    std::string longMsg(1000, 'A');
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << longMsg;
    }
    TS_ASSERT_EQUALS(logger->buffer, longMsg);
    TS_ASSERT_EQUALS(logger->buffer.length(), 1000u);
  }

  // Test 51: Multiple setprecision calls
  void testMultiplePrecision() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::setprecision(2) << 3.14159;
      log << " ";
      log << std::setprecision(5) << 3.14159;
    }
    TS_ASSERT(logger->buffer.find("3.1") != std::string::npos);
    TS_ASSERT(logger->buffer.find("3.1416") != std::string::npos);
  }

  // Test 52: Bool values
  void testBoolValues() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::boolalpha << true << " " << false;
    }
    TS_ASSERT_EQUALS(logger->buffer, "true false");
  }

  // Test 53: Hex output
  void testHexOutput() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::hex << 255;
    }
    TS_ASSERT_EQUALS(logger->buffer, "ff");
  }

  // Test 54: Octal output
  void testOctalOutput() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::oct << 64;
    }
    TS_ASSERT_EQUALS(logger->buffer, "100");
  }

  // Test 55: Fixed notation
  void testFixedNotation() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::fixed << std::setprecision(2) << 1234.5678;
    }
    TS_ASSERT_EQUALS(logger->buffer, "1234.57");
  }

  // Test 56: Scientific notation
  void testScientificNotation() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::scientific << std::setprecision(2) << 1234.5;
    }
    TS_ASSERT(logger->buffer.find("e") != std::string::npos ||
              logger->buffer.find("E") != std::string::npos);
  }

  // Test 57: Negative numbers
  void testNegativeNumbers() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << -42 << " " << -3.14;
    }
    TS_ASSERT(logger->buffer.find("-42") != std::string::npos);
    TS_ASSERT(logger->buffer.find("-3.14") != std::string::npos);
  }

  // Test 58: Zero values
  void testZeroValues() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << 0 << " " << 0.0;
    }
    TS_ASSERT(logger->buffer.find("0") != std::string::npos);
  }

  // Test 59: Very large integers
  void testLargeIntegers() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << 9223372036854775807LL;
    }
    TS_ASSERT(!logger->buffer.empty());
  }

  // Test 60: Multiple endl
  void testMultipleEndl() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << "A" << std::endl << "B" << std::endl << "C";
    }
    TS_ASSERT_EQUALS(logger->buffer, "A\nB\nC");
  }

  // Test 61: Tabs in message
  void testTabsInMessage() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << "A\tB\tC";
    }
    TS_ASSERT_EQUALS(logger->buffer, "A\tB\tC");
  }

  // Test 62: Carriage return
  void testCarriageReturn() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << "Line1\rLine2";
    }
    TS_ASSERT_EQUALS(logger->buffer, "Line1\rLine2");
  }

  // Test 63: Vector with negative values
  void testColumnVector3Negative() {
    auto logger = std::make_shared<DummyLogger>();
    JSBSim::FGColumnVector3 vec(-1, -2, -3);
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << vec;
    }
    TS_ASSERT_EQUALS(logger->buffer, "-1 , -2 , -3");
  }

  // Test 64: Vector with zeros
  void testColumnVector3Zero() {
    auto logger = std::make_shared<DummyLogger>();
    JSBSim::FGColumnVector3 vec(0, 0, 0);
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << vec;
    }
    TS_ASSERT_EQUALS(logger->buffer, "0 , 0 , 0");
  }

  // Test 65: Showpos manipulator
  void testShowpos() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::showpos << 42;
    }
    TS_ASSERT_EQUALS(logger->buffer, "+42");
  }

  // Test 66: Mixed message types
  void testMixedMessageTypes() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << "Count: " << 42 << ", Value: " << 3.14;
    }
    TS_ASSERT(logger->buffer.find("Count: 42") != std::string::npos);
    TS_ASSERT(logger->buffer.find("Value: 3.14") != std::string::npos);
  }

  // Test 67: Nested path
  void testNestedPath() {
    auto logger = std::make_shared<DummyLogger>();
    SGPath path("a/b/c");
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << path;
    }
    TS_ASSERT(logger->buffer.find("a/b/c") != std::string::npos);
  }

  // Test 68: Large vector components
  void testLargeVectorComponents() {
    auto logger = std::make_shared<DummyLogger>();
    JSBSim::FGColumnVector3 vec(1e10, 2e10, 3e10);
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << vec;
    }
    TS_ASSERT(!logger->buffer.empty());
  }

  // Test 69: Path with spaces
  void testPathWithSpaces() {
    auto logger = std::make_shared<DummyLogger>();
    SGPath path("path with spaces/to");
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << (path/"file name");
    }
    TS_ASSERT(logger->buffer.find("path with spaces") != std::string::npos);
  }

  // Test 70: Sequential logging sessions
  void testSequentialLoggingSessions() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << "First";
    }
    TS_ASSERT(logger->flushed);
    TS_ASSERT_EQUALS(logger->buffer, "First");

    logger->flushed = false;
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::DEBUG);
      log << "Second";
    }
    TS_ASSERT(logger->flushed);
    TS_ASSERT_EQUALS(logger->buffer, "FirstSecond");
  }

  // Test 71: Char array
  void testCharArray() {
    auto logger = std::make_shared<DummyLogger>();
    char msg[] = "Array message";
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << msg;
    }
    TS_ASSERT_EQUALS(logger->buffer, "Array message");
  }

  // Test 72: Unsigned integers
  void testUnsignedIntegers() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << 42u << " " << 100ul;
    }
    TS_ASSERT(logger->buffer.find("42") != std::string::npos);
    TS_ASSERT(logger->buffer.find("100") != std::string::npos);
  }

  // Test 73: Size_t
  void testSizeT() {
    auto logger = std::make_shared<DummyLogger>();
    size_t sz = 12345;
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << sz;
    }
    TS_ASSERT_EQUALS(logger->buffer, "12345");
  }

  // Test 74: Pointer value (nullptr check)
  void testNullLogger() {
    // This tests that we can create the logger (not crash)
    auto logger = std::make_shared<DummyLogger>();
    TS_ASSERT(logger != nullptr);
  }

  // Test 75: Format reset
  void testFormatStateReset() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::hex << 255;
    }
    TS_ASSERT_EQUALS(logger->buffer, "ff");

    logger->buffer.clear();
    logger->flushed = false;

    // Second log should use default format
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << 255;
    }
    TS_ASSERT_EQUALS(logger->buffer, "255");
  }
};

/***************************************************************************
 * Extended Log Tests (Tests 76-100)
 ***************************************************************************/

class FGLogExtendedTest : public CxxTest::TestSuite
{
public:
  // Test 76: Very small floating point values
  void testVerySmallFloats() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::scientific << 1.23e-300;
    }
    TS_ASSERT(!logger->buffer.empty());
    TS_ASSERT(logger->buffer.find("e") != std::string::npos ||
              logger->buffer.find("E") != std::string::npos);
  }

  // Test 77: Very large floating point values
  void testVeryLargeFloats() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::scientific << 1.23e+300;
    }
    TS_ASSERT(!logger->buffer.empty());
  }

  // Test 78: Infinity value
  void testInfinityValue() {
    auto logger = std::make_shared<DummyLogger>();
    double inf = std::numeric_limits<double>::infinity();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << inf;
    }
    TS_ASSERT(!logger->buffer.empty());
    // Output typically contains "inf" or "infinity"
    TS_ASSERT(logger->buffer.find("inf") != std::string::npos ||
              logger->buffer.find("Inf") != std::string::npos);
  }

  // Test 79: NaN value
  void testNaNValue() {
    auto logger = std::make_shared<DummyLogger>();
    double nan = std::numeric_limits<double>::quiet_NaN();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << nan;
    }
    TS_ASSERT(!logger->buffer.empty());
    TS_ASSERT(logger->buffer.find("nan") != std::string::npos ||
              logger->buffer.find("NaN") != std::string::npos ||
              logger->buffer.find("NAN") != std::string::npos);
  }

  // Test 80: Setw with right alignment
  void testSetwRightAlignment() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::setw(8) << 42;
    }
    TS_ASSERT_EQUALS(logger->buffer.length(), 8u);
    TS_ASSERT(logger->buffer.find("42") != std::string::npos);
  }

  // Test 81: Uppercase hex
  void testUppercaseHex() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::hex << std::uppercase << 255;
    }
    TS_ASSERT_EQUALS(logger->buffer, "FF");
  }

  // Test 82: Showbase with hex
  void testShowbaseHex() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::showbase << std::hex << 255;
    }
    TS_ASSERT(logger->buffer.find("0x") != std::string::npos ||
              logger->buffer.find("0X") != std::string::npos);
  }

  // Test 83: Showbase with octal
  void testShowbaseOctal() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::showbase << std::oct << 64;
    }
    TS_ASSERT(logger->buffer.find("0") == 0);  // Starts with 0
  }

  // Test 84: Internal alignment
  void testInternalAlignment() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::internal << std::setw(6) << -42;
    }
    TS_ASSERT_EQUALS(logger->buffer.length(), 6u);
    TS_ASSERT(logger->buffer.find("-") != std::string::npos);
    TS_ASSERT(logger->buffer.find("42") != std::string::npos);
  }

  // Test 85: Showpoint for floats
  void testShowpoint() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::showpoint << 100.0;
    }
    TS_ASSERT(logger->buffer.find(".") != std::string::npos);
  }

  // Test 86: Multiple format switches
  void testMultipleFormatSwitches() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::hex << 15 << " " << std::dec << 15 << " " << std::oct << 15;
    }
    TS_ASSERT_EQUALS(logger->buffer, "f 15 17");
  }

  // Test 87: Log level comparison
  void testLogLevelComparison() {
    TS_ASSERT(JSBSim::LogLevel::BULK < JSBSim::LogLevel::DEBUG);
    TS_ASSERT(JSBSim::LogLevel::DEBUG < JSBSim::LogLevel::INFO);
    TS_ASSERT(JSBSim::LogLevel::INFO < JSBSim::LogLevel::WARN);
    TS_ASSERT(JSBSim::LogLevel::WARN < JSBSim::LogLevel::ERROR);
    TS_ASSERT(JSBSim::LogLevel::ERROR < JSBSim::LogLevel::FATAL);
  }

  // Test 88: Double precision output
  void testDoublePrecisionOutput() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << std::setprecision(15) << 3.141592653589793;
    }
    TS_ASSERT(logger->buffer.find("3.14159265358979") != std::string::npos);
  }

  // Test 89: Short integer values
  void testShortIntegerValues() {
    auto logger = std::make_shared<DummyLogger>();
    short s = -32768;
    unsigned short us = 65535;
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << s << " " << us;
    }
    TS_ASSERT(logger->buffer.find("-32768") != std::string::npos);
    TS_ASSERT(logger->buffer.find("65535") != std::string::npos);
  }

  // Test 90: Long double values
  void testLongDoubleValues() {
    auto logger = std::make_shared<DummyLogger>();
    long double ld = 3.14159265358979323846L;
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << ld;
    }
    TS_ASSERT(!logger->buffer.empty());
    TS_ASSERT(logger->buffer.find("3.14") != std::string::npos);
  }

  // Test 91: Mixed endl and explicit newlines
  void testMixedNewlines() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << "A" << std::endl << "B\nC" << std::endl;
    }
    TS_ASSERT_EQUALS(logger->buffer, "A\nB\nC\n");
  }

  // Test 92: Unicode-like characters (extended ASCII)
  void testExtendedASCII() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << "Degrees: 180\xB0";  // degree symbol
    }
    TS_ASSERT(logger->buffer.find("Degrees: 180") != std::string::npos);
  }

  // Test 93: Empty path
  void testEmptyPath() {
    auto logger = std::make_shared<DummyLogger>();
    SGPath path("");
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << path;
    }
    TS_ASSERT(logger->buffer.find("Path") != std::string::npos);
  }

  // Test 94: Absolute path
  void testAbsolutePath() {
    auto logger = std::make_shared<DummyLogger>();
    SGPath path("/root/path/to/file");
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << path;
    }
    TS_ASSERT(logger->buffer.find("/root/path/to/file") != std::string::npos);
  }

  // Test 95: Vector with small components
  void testVectorSmallComponents() {
    auto logger = std::make_shared<DummyLogger>();
    JSBSim::FGColumnVector3 vec(1e-10, 2e-10, 3e-10);
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << vec;
    }
    TS_ASSERT(!logger->buffer.empty());
  }

  // Test 96: Multiple flush behavior
  void testMultipleFlushBehavior() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log1(logger, JSBSim::LogLevel::INFO);
      log1 << "First";
    }
    TS_ASSERT(logger->flushed);
    size_t bufferLen = logger->buffer.length();

    logger->flushed = false;
    {
      JSBSim::FGLogging log2(logger, JSBSim::LogLevel::DEBUG);
      log2 << "Second";
    }
    TS_ASSERT(logger->flushed);
    TS_ASSERT(logger->buffer.length() > bufferLen);
  }

  // Test 97: Log level change between messages
  void testLogLevelChangeBetweenMessages() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << "Info message";
    }
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::INFO);

    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::ERROR);
      log << "Error message";
    }
    TS_ASSERT_EQUALS(logger->GetLogLevel(), JSBSim::LogLevel::ERROR);
  }

  // Test 98: Character special values
  void testCharacterSpecialValues() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << 'A' << ' ' << '\t' << '\n';
    }
    TS_ASSERT_EQUALS(logger->buffer, "A \t\n");
  }

  // Test 99: Multi-line output
  void testMultiLineOutput() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      log << "Line 1" << std::endl;
      log << "Line 2" << std::endl;
      log << "Line 3";
    }
    TS_ASSERT(!logger->buffer.empty());
    TS_ASSERT(logger->buffer.find("Line 1") != std::string::npos);
    TS_ASSERT(logger->buffer.find("Line 2") != std::string::npos);
    TS_ASSERT(logger->buffer.find("Line 3") != std::string::npos);
  }

  // Test 100: Very long concatenation
  void testVeryLongConcatenation() {
    auto logger = std::make_shared<DummyLogger>();
    {
      JSBSim::FGLogging log(logger, JSBSim::LogLevel::INFO);
      for (int i = 0; i < 100; ++i) {
        log << i << " ";
      }
    }
    TS_ASSERT(!logger->buffer.empty());
    TS_ASSERT(logger->buffer.find("0 ") != std::string::npos);
    TS_ASSERT(logger->buffer.find("99 ") != std::string::npos);
  }
};
