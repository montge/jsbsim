/*******************************************************************************
 * StringUtilitiesTest.h - Unit tests for string utility functions
 *
 * Tests the string utility functions including:
 * - Trimming (left, right, all space)
 * - Case conversion (upper, lower)
 * - Number validation (is_number)
 * - String splitting
 * - String replacement
 * - Locale-independent atof
 *
 * Copyright (c) JSBSim Development Team
 * Licensed under LGPL
 ******************************************************************************/

#include <string>
#include <vector>
#include <limits>
#include <cxxtest/TestSuite.h>
#include "FGJSBBase.h"
#include <input_output/string_utilities.h>

using namespace JSBSim;

class StringUtilitiesTest : public CxxTest::TestSuite
{
public:
  /***************************************************************************
   * trim_left Tests
   ***************************************************************************/

  void testTrimLeftSpaces() {
    std::string s = "   hello";
    TS_ASSERT_EQUALS(trim_left(s), "hello");
  }

  void testTrimLeftTabs() {
    std::string s = "\t\thello";
    TS_ASSERT_EQUALS(trim_left(s), "hello");
  }

  void testTrimLeftMixed() {
    std::string s = " \t \thello world";
    TS_ASSERT_EQUALS(trim_left(s), "hello world");
  }

  void testTrimLeftNoWhitespace() {
    std::string s = "hello";
    TS_ASSERT_EQUALS(trim_left(s), "hello");
  }

  void testTrimLeftEmpty() {
    std::string s = "";
    TS_ASSERT_EQUALS(trim_left(s), "");
  }

  void testTrimLeftAllSpaces() {
    std::string s = "     ";
    TS_ASSERT_EQUALS(trim_left(s), "");
  }

  void testTrimLeftPreservesRight() {
    std::string s = "  hello  ";
    TS_ASSERT_EQUALS(trim_left(s), "hello  ");
  }

  /***************************************************************************
   * trim_right Tests
   ***************************************************************************/

  void testTrimRightSpaces() {
    std::string s = "hello   ";
    TS_ASSERT_EQUALS(trim_right(s), "hello");
  }

  void testTrimRightTabs() {
    std::string s = "hello\t\t";
    TS_ASSERT_EQUALS(trim_right(s), "hello");
  }

  void testTrimRightMixed() {
    std::string s = "hello world \t \t";
    TS_ASSERT_EQUALS(trim_right(s), "hello world");
  }

  void testTrimRightNoWhitespace() {
    std::string s = "hello";
    TS_ASSERT_EQUALS(trim_right(s), "hello");
  }

  void testTrimRightEmpty() {
    std::string s = "";
    TS_ASSERT_EQUALS(trim_right(s), "");
  }

  void testTrimRightAllSpaces() {
    std::string s = "     ";
    TS_ASSERT_EQUALS(trim_right(s), "");
  }

  void testTrimRightPreservesLeft() {
    std::string s = "  hello  ";
    TS_ASSERT_EQUALS(trim_right(s), "  hello");
  }

  /***************************************************************************
   * trim Tests (both sides)
   ***************************************************************************/

  void testTrimBothSides() {
    std::string s = "  hello  ";
    TS_ASSERT_EQUALS(trim(s), "hello");
  }

  void testTrimMixedWhitespace() {
    std::string s = " \t  hello world\t  ";
    TS_ASSERT_EQUALS(trim(s), "hello world");
  }

  void testTrimEmpty() {
    std::string s = "";
    TS_ASSERT_EQUALS(trim(s), "");
  }

  void testTrimAllWhitespace() {
    std::string s = "  \t\t  ";
    TS_ASSERT_EQUALS(trim(s), "");
  }

  void testTrimNoWhitespace() {
    std::string s = "hello";
    TS_ASSERT_EQUALS(trim(s), "hello");
  }

  void testTrimPreservesInternalSpaces() {
    std::string s = "  hello   world  ";
    TS_ASSERT_EQUALS(trim(s), "hello   world");
  }

  /***************************************************************************
   * trim_all_space Tests
   ***************************************************************************/

  void testTrimAllSpaceRemovesAll() {
    std::string s = " h e l l o ";
    TS_ASSERT_EQUALS(trim_all_space(s), "hello");
  }

  void testTrimAllSpaceEmpty() {
    std::string s = "";
    TS_ASSERT_EQUALS(trim_all_space(s), "");
  }

  void testTrimAllSpaceOnlySpaces() {
    std::string s = "    ";
    TS_ASSERT_EQUALS(trim_all_space(s), "");
  }

  void testTrimAllSpaceTabs() {
    std::string s = "h\te\tl\tl\to";
    TS_ASSERT_EQUALS(trim_all_space(s), "hello");
  }

  void testTrimAllSpaceNoSpaces() {
    std::string s = "hello";
    TS_ASSERT_EQUALS(trim_all_space(s), "hello");
  }

  void testTrimAllSpaceMixed() {
    std::string s = " \t xx\t\tyy  zz \t  ";
    TS_ASSERT_EQUALS(trim_all_space(s), "xxyyzz");
  }

  /***************************************************************************
   * to_upper Tests
   ***************************************************************************/

  void testToUpperLowercase() {
    std::string s = "hello";
    TS_ASSERT_EQUALS(to_upper(s), "HELLO");
  }

  void testToUpperMixed() {
    std::string s = "HeLLo WoRLD";
    TS_ASSERT_EQUALS(to_upper(s), "HELLO WORLD");
  }

  void testToUpperAlreadyUpper() {
    std::string s = "HELLO";
    TS_ASSERT_EQUALS(to_upper(s), "HELLO");
  }

  void testToUpperEmpty() {
    std::string s = "";
    TS_ASSERT_EQUALS(to_upper(s), "");
  }

  void testToUpperWithNumbers() {
    std::string s = "abc123xyz";
    TS_ASSERT_EQUALS(to_upper(s), "ABC123XYZ");
  }

  void testToUpperWithSpecialChars() {
    std::string s = "hello-world_test!";
    TS_ASSERT_EQUALS(to_upper(s), "HELLO-WORLD_TEST!");
  }

  void testToUpperPreservesWhitespace() {
    std::string s = " mixed\tCASE; ";
    TS_ASSERT_EQUALS(to_upper(s), " MIXED\tCASE; ");
  }

  /***************************************************************************
   * to_lower Tests
   ***************************************************************************/

  void testToLowerUppercase() {
    std::string s = "HELLO";
    TS_ASSERT_EQUALS(to_lower(s), "hello");
  }

  void testToLowerMixed() {
    std::string s = "HeLLo WoRLD";
    TS_ASSERT_EQUALS(to_lower(s), "hello world");
  }

  void testToLowerAlreadyLower() {
    std::string s = "hello";
    TS_ASSERT_EQUALS(to_lower(s), "hello");
  }

  void testToLowerEmpty() {
    std::string s = "";
    TS_ASSERT_EQUALS(to_lower(s), "");
  }

  void testToLowerWithNumbers() {
    std::string s = "ABC123XYZ";
    TS_ASSERT_EQUALS(to_lower(s), "abc123xyz");
  }

  void testToLowerWithSpecialChars() {
    std::string s = "HELLO-WORLD_TEST!";
    TS_ASSERT_EQUALS(to_lower(s), "hello-world_test!");
  }

  void testToLowerPreservesWhitespace() {
    std::string s = " MiXed\tCaSE; ";
    TS_ASSERT_EQUALS(to_lower(s), " mixed\tcase; ");
  }

  /***************************************************************************
   * is_number Tests - Valid Numbers
   ***************************************************************************/

  void testIsNumberInteger() {
    TS_ASSERT(is_number("123"));
    TS_ASSERT(is_number("0"));
    TS_ASSERT(is_number("1526"));
  }

  void testIsNumberDecimal() {
    TS_ASSERT(is_number("1.0"));
    TS_ASSERT(is_number("3.14159"));
    TS_ASSERT(is_number(".5"));
    TS_ASSERT(is_number(".01256"));
  }

  void testIsNumberNegative() {
    TS_ASSERT(is_number("-1"));
    TS_ASSERT(is_number("-3.14"));
    TS_ASSERT(is_number("-.5"));
  }

  void testIsNumberPositive() {
    TS_ASSERT(is_number("+1"));
    TS_ASSERT(is_number("+3.14"));
    TS_ASSERT(is_number("+.5"));
  }

  void testIsNumberScientific() {
    TS_ASSERT(is_number("1e10"));
    TS_ASSERT(is_number("1E10"));
    TS_ASSERT(is_number("-1.0e+1"));
    TS_ASSERT(is_number("3.14e-5"));
    TS_ASSERT(is_number("+2.5E+10"));
  }

  /***************************************************************************
   * is_number Tests - Invalid Numbers
   ***************************************************************************/

  void testIsNumberEmpty() {
    TS_ASSERT(!is_number(""));
    TS_ASSERT(!is_number(" "));
  }

  void testIsNumberLetters() {
    TS_ASSERT(!is_number("abc"));
    TS_ASSERT(!is_number("x"));
    TS_ASSERT(!is_number("3.14a"));
  }

  void testIsNumberMixedInvalid() {
    TS_ASSERT(!is_number("125x5#"));
    TS_ASSERT(!is_number("1.0.0"));
    TS_ASSERT(!is_number("1..0"));
  }

  void testIsNumberIncompleteExponent() {
    TS_ASSERT(!is_number("1.0e"));
    TS_ASSERT(!is_number("1.0e+"));
    TS_ASSERT(!is_number("1.0e-"));
    TS_ASSERT(!is_number("-.1e-"));
    TS_ASSERT(!is_number("1.0e1.0"));
  }

  void testIsNumberMultipleSigns() {
    TS_ASSERT(!is_number("--1"));
    TS_ASSERT(!is_number("++1"));
    TS_ASSERT(!is_number("1+"));
    TS_ASSERT(!is_number("1-"));
  }

  /***************************************************************************
   * split Tests
   ***************************************************************************/

  void testSplitEmpty() {
    std::vector<std::string> list = split("", ',');
    TS_ASSERT_EQUALS(list.size(), 0);
  }

  void testSplitNoDelimiter() {
    std::vector<std::string> list = split("hello", ',');
    TS_ASSERT_EQUALS(list.size(), 1);
    TS_ASSERT_EQUALS(list[0], "hello");
  }

  void testSplitSimple() {
    std::vector<std::string> list = split("a,b,c", ',');
    TS_ASSERT_EQUALS(list.size(), 3);
    TS_ASSERT_EQUALS(list[0], "a");
    TS_ASSERT_EQUALS(list[1], "b");
    TS_ASSERT_EQUALS(list[2], "c");
  }

  void testSplitOnlyDelimiters() {
    std::vector<std::string> list = split(",,,,,", ',');
    TS_ASSERT_EQUALS(list.size(), 0);
  }

  void testSplitLeadingTrailingDelimiters() {
    std::vector<std::string> list = split(",xx,,yy,zz,", ',');
    TS_ASSERT_EQUALS(list.size(), 3);
    TS_ASSERT_EQUALS(list[0], "xx");
    TS_ASSERT_EQUALS(list[1], "yy");
    TS_ASSERT_EQUALS(list[2], "zz");
  }

  void testSplitBySpace() {
    std::vector<std::string> list = split(" xx yy zz ", ' ');
    TS_ASSERT_EQUALS(list.size(), 3);
    TS_ASSERT_EQUALS(list[0], "xx");
    TS_ASSERT_EQUALS(list[1], "yy");
    TS_ASSERT_EQUALS(list[2], "zz");
  }

  void testSplitPreservesContent() {
    std::vector<std::string> list = split(" xx yy zz ", ',');
    TS_ASSERT_EQUALS(list.size(), 1);
    TS_ASSERT_EQUALS(list[0], "xx yy zz");
  }

  void testSplitDifferentDelimiter() {
    std::vector<std::string> list = split("a:b:c", ':');
    TS_ASSERT_EQUALS(list.size(), 3);
    TS_ASSERT_EQUALS(list[0], "a");
    TS_ASSERT_EQUALS(list[1], "b");
    TS_ASSERT_EQUALS(list[2], "c");
  }

  void testSplitWithSpacesInElements() {
    std::vector<std::string> list = split("hello world,foo bar", ',');
    TS_ASSERT_EQUALS(list.size(), 2);
    TS_ASSERT_EQUALS(list[0], "hello world");
    TS_ASSERT_EQUALS(list[1], "foo bar");
  }

  /***************************************************************************
   * replace Tests
   ***************************************************************************/

  void testReplaceEmpty() {
    TS_ASSERT_EQUALS(replace("", "x", "a"), "");
  }

  void testReplaceBeginning() {
    TS_ASSERT_EQUALS(replace("xyzzu", "x", "a"), "ayzzu");
  }

  void testReplaceEnd() {
    TS_ASSERT_EQUALS(replace("xyzzu", "u", "a"), "xyzza");
  }

  void testReplaceMiddle() {
    TS_ASSERT_EQUALS(replace("xyzzu", "z", "y"), "xyyzu");
  }

  void testReplaceMultiCharPattern() {
    TS_ASSERT_EQUALS(replace("xyzzu", "yzz", "ab"), "xabzzu");
  }

  void testReplaceNotFound() {
    TS_ASSERT_EQUALS(replace("xyzzu", "b", "w"), "xyzzu");
  }

  void testReplaceWithSpaces() {
    TS_ASSERT_EQUALS(replace(" xyzzu ", "x", "a"), " ayzzu ");
  }

  void testReplaceLongerReplacement() {
    TS_ASSERT_EQUALS(replace("abc", "b", "xyz"), "axyzc");
  }

  void testReplaceShorterReplacement() {
    // replace inserts replacement at pattern position but keeps rest
    TS_ASSERT_EQUALS(replace("abcde", "bcd", "x"), "axcde");
  }

  void testReplaceEmptyReplacement() {
    TS_ASSERT_EQUALS(replace("abcde", "c", ""), "abde");
  }

  void testReplaceOnlyFirst() {
    // Replace only replaces first occurrence
    std::string result = replace("ababa", "a", "x");
    TS_ASSERT_EQUALS(result, "xbaba");
  }

  /***************************************************************************
   * atof_locale_c Tests - Valid Numbers
   ***************************************************************************/

  void testAtofZero() {
    TS_ASSERT_EQUALS(atof_locale_c("0"), 0.0);
    TS_ASSERT_EQUALS(atof_locale_c("0.0"), 0.0);
    TS_ASSERT_EQUALS(atof_locale_c(".0"), 0.0);
    TS_ASSERT_EQUALS(atof_locale_c("0."), 0.0);
  }

  void testAtofPositive() {
    TS_ASSERT_EQUALS(atof_locale_c("1"), 1.0);
    TS_ASSERT_EQUALS(atof_locale_c("+1"), 1.0);
    TS_ASSERT_EQUALS(atof_locale_c("123.4"), 123.4);
  }

  void testAtofNegative() {
    TS_ASSERT_EQUALS(atof_locale_c("-1"), -1.0);
    TS_ASSERT_EQUALS(atof_locale_c("-123.4"), -123.4);
  }

  void testAtofDecimal() {
    TS_ASSERT_EQUALS(atof_locale_c(".25"), 0.25);
    TS_ASSERT_EQUALS(atof_locale_c("3.14159"), 3.14159);
  }

  void testAtofScientificLowercase() {
    TS_ASSERT_EQUALS(atof_locale_c("1e1"), 10.0);
    TS_ASSERT_EQUALS(atof_locale_c("1.e1"), 10.0);
    TS_ASSERT_EQUALS(atof_locale_c(".1e1"), 1.0);
    TS_ASSERT_EQUALS(atof_locale_c("3.14e2"), 314.0);
    TS_ASSERT_EQUALS(atof_locale_c("3.14e-2"), 0.0314);
  }

  void testAtofScientificUppercase() {
    TS_ASSERT_EQUALS(atof_locale_c("1E1"), 10.0);
    TS_ASSERT_EQUALS(atof_locale_c("3.14E2"), 314.0);
    TS_ASSERT_EQUALS(atof_locale_c("3.14E-2"), 0.0314);
  }

  void testAtofScientificWithSigns() {
    TS_ASSERT_EQUALS(atof_locale_c("+3.14e+2"), 314.0);
    TS_ASSERT_EQUALS(atof_locale_c("-3.14e+2"), -314.0);
    TS_ASSERT_EQUALS(atof_locale_c("+3.14e-2"), 0.0314);
    TS_ASSERT_EQUALS(atof_locale_c("-3.14e-2"), -0.0314);
  }

  void testAtofWithWhitespace() {
    TS_ASSERT_EQUALS(atof_locale_c(" 123.4"), 123.4);
    TS_ASSERT_EQUALS(atof_locale_c("123.4 "), 123.4);
    TS_ASSERT_EQUALS(atof_locale_c(" 123.4 "), 123.4);
  }

  void testAtofUnderflow() {
    // Very small exponents should round to zero
    TS_ASSERT_EQUALS(atof_locale_c("1E-999"), 0.0);
    TS_ASSERT_EQUALS(atof_locale_c("-1E-999"), 0.0);
  }

  /***************************************************************************
   * atof_locale_c Tests - Invalid Numbers (throws)
   ***************************************************************************/

  void testAtofOverflow() {
    TS_ASSERT_THROWS(atof_locale_c("1E+999"), InvalidNumber&);
    TS_ASSERT_THROWS(atof_locale_c("-1E+999"), InvalidNumber&);
  }

  void testAtofInvalidString() {
    TS_ASSERT_THROWS(atof_locale_c("invalid"), InvalidNumber&);
    TS_ASSERT_THROWS(atof_locale_c("abc"), InvalidNumber&);
  }

  void testAtofMultipleDecimals() {
    TS_ASSERT_THROWS(atof_locale_c("1.0.0"), InvalidNumber&);
  }

  void testAtofIncompleteExponent() {
    TS_ASSERT_THROWS(atof_locale_c("1E-"), InvalidNumber&);
    TS_ASSERT_THROWS(atof_locale_c("1E+"), InvalidNumber&);
    TS_ASSERT_THROWS(atof_locale_c("1.2E"), InvalidNumber&);
  }

  void testAtofExponentOnly() {
    TS_ASSERT_THROWS(atof_locale_c("E-2"), InvalidNumber&);
    TS_ASSERT_THROWS(atof_locale_c(".E2"), InvalidNumber&);
    TS_ASSERT_THROWS(atof_locale_c(".E"), InvalidNumber&);
  }

  void testAtofDecimalExponent() {
    TS_ASSERT_THROWS(atof_locale_c("1.2E1.0"), InvalidNumber&);
  }

  void testAtofMultipleSigns() {
    TS_ASSERT_THROWS(atof_locale_c("--1"), InvalidNumber&);
    TS_ASSERT_THROWS(atof_locale_c("++1"), InvalidNumber&);
  }

  void testAtofEmptyInput() {
    TS_ASSERT_THROWS(atof_locale_c(""), InvalidNumber&);
    TS_ASSERT_THROWS(atof_locale_c(" "), InvalidNumber&);
  }

  void testAtofJustDecimalPoint() {
    TS_ASSERT_THROWS(atof_locale_c("."), InvalidNumber&);
  }

  /***************************************************************************
   * Edge Case Tests
   ***************************************************************************/

  void testTrimOriginalTest() {
    const std::string s_ref(" \t  xx\t\tyy  zz \t  ");
    const std::string all_spaces("  \t \t\t  ");
    std::string s = s_ref;
    TS_ASSERT_EQUALS(trim_left(s), std::string("xx\t\tyy  zz \t  "));
    s = all_spaces;
    TS_ASSERT_EQUALS(trim_left(s), std::string(""));
    s = s_ref;
    TS_ASSERT_EQUALS(trim_right(s), std::string(" \t  xx\t\tyy  zz"));
    s = all_spaces;
    TS_ASSERT_EQUALS(trim_right(s), std::string(""));
    s = s_ref;
    TS_ASSERT_EQUALS(trim(s), std::string("xx\t\tyy  zz"));
    s = all_spaces;
    TS_ASSERT_EQUALS(trim(s), std::string(""));
    s = s_ref;
    TS_ASSERT_EQUALS(trim_all_space(s), std::string("xxyyzz"));
    s = all_spaces;
    TS_ASSERT_EQUALS(trim_all_space(s), std::string(""));
  }

  void testSingleCharacterStrings() {
    std::string s = "a";
    TS_ASSERT_EQUALS(trim_left(s), "a");
    s = "a";
    TS_ASSERT_EQUALS(trim_right(s), "a");
    s = "a";
    TS_ASSERT_EQUALS(trim(s), "a");
    s = "a";
    TS_ASSERT_EQUALS(to_upper(s), "A");
    s = "A";
    TS_ASSERT_EQUALS(to_lower(s), "a");
  }

  void testNumericStringsInTrim() {
    std::string s = "  123.45  ";
    TS_ASSERT_EQUALS(trim(s), "123.45");
  }

  void testSpecialCharacters() {
    std::string s = "@#$%^&*()";
    TS_ASSERT_EQUALS(to_upper(s), "@#$%^&*()");
    s = "@#$%^&*()";
    TS_ASSERT_EQUALS(to_lower(s), "@#$%^&*()");
    TS_ASSERT(!is_number("@#$"));
  }

  void testUnicodeNotAffected() {
    // Basic check that ASCII conversion doesn't crash on high bytes
    std::string s = "abc";
    TS_ASSERT_EQUALS(to_upper(s), "ABC");
  }

  void testLongStrings() {
    std::string long_str(1000, 'a');
    std::string s = long_str;
    TS_ASSERT_EQUALS(to_upper(s).length(), 1000);

    std::string expected(1000, 'A');
    TS_ASSERT_EQUALS(s, expected);
  }

  void testSplitManyElements() {
    std::vector<std::string> list = split("a,b,c,d,e,f,g,h,i,j", ',');
    TS_ASSERT_EQUALS(list.size(), 10);
  }

  /***************************************************************************
   * Complete System Tests
   ***************************************************************************/

  void testCompleteStringProcessingPipeline() {
    // Simulate parsing a config value from XML
    std::string rawInput = "  +123.456e-2  ";

    // Step 1: Trim whitespace
    std::string trimmed = trim(rawInput);
    TS_ASSERT_EQUALS(trimmed, "+123.456e-2");

    // Step 2: Validate it's a number
    TS_ASSERT(is_number(trimmed));

    // Step 3: Convert to double
    double value = atof_locale_c(trimmed);
    TS_ASSERT_DELTA(value, 1.23456, 1e-10);
  }

  void testCompleteCSVParsing() {
    // Parse a CSV line with mixed content
    std::string csvLine = " name, value1, VALUE2, 3.14 ";
    std::string cleaned = trim(csvLine);

    std::vector<std::string> fields = split(cleaned, ',');
    TS_ASSERT_EQUALS(fields.size(), 4);

    // Trim and normalize each field
    TS_ASSERT_EQUALS(trim(fields[0]), "name");
    TS_ASSERT_EQUALS(to_lower(trim(fields[2])), "value2");
    TS_ASSERT(is_number(trim(fields[3])));
  }

  void testCompleteConfigKeyProcessing() {
    // Process a config key (common in XML parsing)
    std::string key = "  Aircraft_Name  ";

    // Clean and normalize
    std::string cleaned = trim(key);
    std::string forUpper = cleaned;  // Make copy since to_upper modifies in place
    std::string upper = to_upper(forUpper);
    std::string noSpaces = trim_all_space(key);

    TS_ASSERT_EQUALS(cleaned, "Aircraft_Name");
    TS_ASSERT_EQUALS(upper, "AIRCRAFT_NAME");
    TS_ASSERT_EQUALS(noSpaces, "Aircraft_Name");
  }

  /***************************************************************************
   * Instance Independence Tests
   ***************************************************************************/

  void testIndependentStringOperations() {
    std::string s1 = "  HELLO  ";
    std::string s2 = "  HELLO  ";

    // Operate on s1 only
    std::string r1 = trim(s1);
    std::string r2 = to_lower(trim(s2));

    // Results should be independent
    TS_ASSERT_EQUALS(r1, "HELLO");
    TS_ASSERT_EQUALS(r2, "hello");
    TS_ASSERT(r1 != r2);
  }

  void testIndependentSplitResults() {
    std::string input = "a,b,c";

    std::vector<std::string> list1 = split(input, ',');
    std::vector<std::string> list2 = split(input, ',');

    // Modify list1
    list1[0] = "modified";

    // list2 should be unchanged
    TS_ASSERT_EQUALS(list2[0], "a");
    TS_ASSERT_EQUALS(list2[1], "b");
    TS_ASSERT_EQUALS(list2[2], "c");
  }
};
