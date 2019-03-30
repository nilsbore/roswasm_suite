// CBOR-lite Test Cases
// This file is part of CBOR-lite which is copyright Isode Limited
// and others and released under a MIT license. For details, see the
// COPYRIGHT.md file in the top-level folder of the CBOR-lite software
// distribution.
#include "../include/cbor-lite/codec.h"
#include <boost/test/unit_test.hpp>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <limits>
#include <tuple>

BOOST_AUTO_TEST_SUITE(cbor)

BOOST_AUTO_TEST_CASE(base) {
    std::vector<std::pair<std::size_t, std::size_t>> cases{
        {0, 0},
        {1, 0},
        {23, 0},
        {24, 1},
        {255, 1},
        {256, 2},
        {65535, 2},
        {65536, 4},
        {4294967295, 4},
        {4294967296, 8},
    };
    for (const auto& test : cases) {
        std::size_t len;
        BOOST_REQUIRE_NO_THROW(len = CborLite::length(test.first));
        BOOST_TEST(len == test.second, "CBOR length for " << test.first << " is " << len << " not " << test.second);
    }
}

BOOST_AUTO_TEST_CASE(non_negative) {
    const std::vector<std::pair<std::uint_fast64_t, std::string>> cases{
        {0u, std::string("\x00", 1)},
        {1u, "\x01"},
        {10u, "\x0a"},
        {23u, "\x17"},
        {24u, "\x18\x18"},
        {25u, "\x18\x19"},
        {100u, "\x18\x64"},
        {1000u, "\x19\x03\xe8"},
        {1000000u, std::string("\x1a\x00\x0f\x42\x40", 5)},
        {1000000000000u, std::string("\x1b\x00\x00\x00\xe8\xd4\xa5\x10\x00", 9)},
        {18446744073709551615u, std::string("\x1b\xff\xff\xff\xff\xff\xff\xff\xff", 9)},
    };
    for (const auto& test : cases) {
        std::string buffer;
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeUnsigned(buffer, test.first));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(buffer, test.second);
        std::uint_fast64_t value = 0u;
        auto pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeUnsigned(pos, std::end(test.second), value));
        BOOST_CHECK(pos == std::end(test.second));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(value, test.first);
    }
}

BOOST_AUTO_TEST_CASE(negative) {
    const std::vector<std::pair<std::uint_fast64_t, std::string>> cases{
        {0, "\x20"},
        {9, "\x29"},
        {99, "\x38\x63"},
        {999, "\x39\x03\xe7"},
    };
    for (const auto& test : cases) {
        std::string buffer;
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeNegative(buffer, test.first));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(buffer, test.second);
        std::uint_fast64_t value = 0u;
        auto pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeNegative(pos, std::end(test.second), value));
        BOOST_CHECK(pos == std::end(test.second));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(value, test.first);
    }
}

BOOST_AUTO_TEST_CASE(integer) {
    const std::vector<std::pair<std::int_fast64_t, std::string>> cases{
        {0, std::string("\x00", 1)},
        {1, "\x01"},
        {10, "\x0a"},
        {23, "\x17"},
        {24, "\x18\x18"},
        {25, "\x18\x19"},
        {100, "\x18\x64"},
        {1000, "\x19\x03\xe8"},
        {1000000, std::string("\x1a\x00\x0f\x42\x40", 5)},
        {1000000000000, std::string("\x1b\x00\x00\x00\xe8\xd4\xa5\x10\x00", 9)},
        {-1, "\x20"},
        {-10, "\x29"},
        {-100, "\x38\x63"},
        {-1000, "\x39\x03\xe7"},
    };
    for (const auto& test : cases) {
        std::string buffer;
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeInteger(buffer, test.first));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(buffer, test.second);
        std::int_fast64_t value = 0;
        auto pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeInteger(pos, std::end(test.second), value));
        BOOST_CHECK(pos == std::end(test.second));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(value, test.first);
    }
}

BOOST_AUTO_TEST_CASE(no_content) {
    const std::vector<std::pair<bool, std::string>> cases{
        {false, "\xf4"},
        {true, "\xf5"},
    };
    for (const auto& test : cases) {
        std::string buffer;
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeBool(buffer, test.first));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(buffer, test.second);
        auto value = false;
        auto pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeBool(pos, std::end(test.second), value));
        BOOST_CHECK(pos == std::end(test.second));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(value, test.first);
    }
}

BOOST_AUTO_TEST_CASE(bytes) {
    const std::vector<std::pair<std::string, std::string>> cases{
        {"", "\x40"},
        {"a", "\x41\x61"},
        {"A", "\x41\x41"},
        {"IETF", "\x44\x49\x45\x54\x46"},
        {"\"\\", "\x42\x22\x5c"},
        {"\xc3\xbc", "\x42\xc3\xbc"},
        {"\xe6\xb0\xb4", "\x43\xe6\xb0\xb4"},
        {"\xf0\x90\x85\x91", "\x44\xf0\x90\x85\x91"},
        {"\x01\x02\x03\x04", "\x44\x01\x02\x03\x04"},
        {"@@@@", "\x44\x40\x40\x40\x40"},
    };
    for (const auto& test : cases) {
        std::string buffer;
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeBytes(buffer, test.first));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(buffer, test.second);
        std::string value;
        auto pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeBytes(pos, std::end(test.second), value));
        BOOST_CHECK(pos == std::end(test.second));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(value, test.first);
    }
    {
        std::vector<char> buffer;
        std::string input = "@@@@";
        std::vector<char> payload(std::begin(input), std::end(input));
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeBytes(buffer, payload));
        std::string expect = "\x44\x40\x40\x40\x40";
        BOOST_CHECK_EQUAL(len, expect.length());
        BOOST_CHECK_EQUAL(std::string(std::begin(buffer), std::end(buffer)), expect);
    }
}

BOOST_AUTO_TEST_CASE(encodedBytes) {
    const std::vector<std::pair<std::string, std::string>> cases{
        {"", "\xd8\x18\x40"},
        {"a", "\xd8\x18\x41\x61"},
        {"A", "\xd8\x18\x41\x41"},
        {"IETF", "\xd8\x18\x44\x49\x45\x54\x46"},
        {"\"\\", "\xd8\x18\x42\x22\x5c"},
        {"\xc3\xbc", "\xd8\x18\x42\xc3\xbc"},
        {"\xe6\xb0\xb4", "\xd8\x18\x43\xe6\xb0\xb4"},
        {"\xf0\x90\x85\x91", "\xd8\x18\x44\xf0\x90\x85\x91"},
        {"\x01\x02\x03\x04", "\xd8\x18\x44\x01\x02\x03\x04"},
        {"@@@@", "\xd8\x18\x44\x40\x40\x40\x40"},
    };
    for (const auto& test : cases) {
        std::string buffer;
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeEncodedBytes(buffer, test.first));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(buffer, test.second);

        buffer.clear();
        BOOST_CHECK_NO_THROW(len = CborLite::encodeEncodedBytesPrefix(buffer, test.first.length()));
        BOOST_CHECK_EQUAL(len, 3);
        BOOST_CHECK_EQUAL(buffer, test.second.substr(0, 3));

        std::string value;
        auto pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeEncodedBytes(pos, std::end(test.second), value));
        BOOST_CHECK(pos == std::end(test.second));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(value, test.first);

        std::size_t got = 0u;
        pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeEncodedBytesPrefix(pos, pos + 3, got));
        BOOST_CHECK_EQUAL(len, 3);
        BOOST_CHECK_EQUAL(got, test.first.length());
    }
    {
        std::vector<char> buffer;
        std::string input = "@@@@";
        std::vector<char> payload(std::begin(input), std::end(input));
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeEncodedBytes(buffer, payload));
        std::string expect = "\xd8\x18\x44\x40\x40\x40\x40";
        BOOST_CHECK_EQUAL(len, expect.length());
        BOOST_CHECK_EQUAL(std::string(std::begin(buffer), std::end(buffer)), expect);
    }
}

BOOST_AUTO_TEST_CASE(strings) {
    const std::vector<std::pair<std::string, std::string>> cases{
        {"", "\x60"},
        {"a", "\x61\x61"},
        {"A", "\x61\x41"},
        {"IETF", "\x64\x49\x45\x54\x46"},
        {"\"\\", "\x62\x22\x5c"},
        {"\xc3\xbc", "\x62\xc3\xbc"},
        {"\xe6\xb0\xb4", "\x63\xe6\xb0\xb4"},
        {"\xf0\x90\x85\x91", "\x64\xf0\x90\x85\x91"},
        {"\x01\x02\x03\x04", "\x64\x01\x02\x03\x04"},
    };
    for (const auto& test : cases) {
        std::string buffer;
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeText(buffer, test.first));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(buffer, test.second);
        std::string value;
        auto pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeText(pos, std::end(test.second), value));
        BOOST_CHECK(pos == std::end(test.second));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(value, test.first);
    }
}

BOOST_AUTO_TEST_CASE(array) {
    const std::vector<std::pair<std::uint_fast64_t, std::string>> cases{
        {0u, "\x80"},
        {1u, "\x81"},
        {10u, "\x8a"},
        {23u, "\x97"},
        {24u, "\x98\x18"},
        {25u, "\x98\x19"},
        {100u, "\x98\x64"},
        {1000u, "\x99\x03\xe8"},
        {1000000u, std::string("\x9a\x00\x0f\x42\x40", 5)},
        {1000000000000u, std::string("\x9b\x00\x00\x00\xe8\xd4\xa5\x10\x00", 9)},
        {18446744073709551615u, std::string("\x9b\xff\xff\xff\xff\xff\xff\xff\xff", 9)},
    };
    for (const auto& test : cases) {
        std::string buffer;
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeArraySize(buffer, test.first));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(buffer, test.second);
        unsigned long long value = 0;
        auto pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeArraySize(pos, std::end(test.second), value));
        BOOST_CHECK(pos == std::end(test.second));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(value, test.first);
    }
}

BOOST_AUTO_TEST_CASE(map) {
    const std::vector<std::pair<std::uint_fast64_t, std::string>> cases{
        {0u, "\xa0"},
        {1u, "\xa1"},
        {10u, "\xaa"},
        {23u, "\xb7"},
        {24u, "\xb8\x18"},
        {25u, "\xb8\x19"},
        {100u, "\xb8\x64"},
        {1000u, "\xb9\x03\xe8"},
        {1000000u, std::string("\xba\x00\x0f\x42\x40", 5)},
        {1000000000000u, std::string("\xbb\x00\x00\x00\xe8\xd4\xa5\x10\x00", 9)},
        {18446744073709551615u, std::string("\xbb\xff\xff\xff\xff\xff\xff\xff\xff", 9)},
    };
    for (const auto& test : cases) {
        std::string buffer;
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeMapSize(buffer, test.first));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(buffer, test.second);
        unsigned long long value = 0;
        auto pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeMapSize(pos, std::end(test.second), value));
        BOOST_CHECK(pos == std::end(test.second));
        BOOST_CHECK_EQUAL(len, test.second.size());
        BOOST_CHECK_EQUAL(value, test.first);
    }
}

BOOST_AUTO_TEST_CASE(singlef) {
    const std::vector<std::pair<float, std::string>> cases {
        {0.0f, std::string("\xfa\x00\x00\x00\x00", 5)}, {-0.0f, std::string("\xfa\x80\x00\x00\x00", 5)},
            {1.0f, std::string("\xfa\x3f\x80\x00\x00", 5)}, {1.1f, std::string("\xfa\x3f\x8c\xcc\xcd", 5)},
            {1.5f, std::string("\xfa\x3f\xc0\x00\x00", 5)}, {65504.0f, std::string("\xfa\x47\x7f\xe0\x00", 5)},
            {3.4028234663852886e+38f, std::string("\xfa\x7f\x7f\xff\xff", 5)},
#if 0
            {1.0e+300f, std::string("\xfa\x7f\x80\x00\x00", 5)}, // too large for single
#endif
            {5.960464477539063e-8f, std::string("\xfa\x33\x80\x00\x00", 5)},
            {0.00006103515625f, std::string("\xfa\x38\x80\x00\x00", 5)}, {-4.0f, std::string("\xfa\xc0\x80\x00\x00", 5)},
            {-4.1f, std::string("\xfa\xc0\x83\x33\x33", 5)},
            {std::numeric_limits<float>::infinity(), std::string("\xfa\x7f\x80\x00\x00", 5)},
            {std::numeric_limits<float>::quiet_NaN(), std::string("\xfa\x7f\xc0\x00\x00", 5)},
            {-std::numeric_limits<float>::infinity(), std::string("\xfa\xff\x80\x00\x00", 5)},
    };
    for (const auto& test : cases) {
        std::string buffer;
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeSingleFloat(buffer, test.first));
        BOOST_CHECK_EQUAL(len, test.second.size());
#if 0
        std::cout << "Encoding: ";
        for (unsigned ch : buffer) {
            std::cout << "\\x" << std::hex << std::setw(2) << std::setfill('0') << (ch&0xFFu);
        }
        std::cout << std::endl;
#endif
        BOOST_CHECK_EQUAL(buffer, test.second);
        float value = 0;
        auto pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeSingleFloat(pos, std::end(test.second), value));
        BOOST_CHECK(pos == std::end(test.second));
        BOOST_CHECK_EQUAL(len, test.second.size());
        if (std::isnan(test.first)) {
            BOOST_CHECK(std::isnan(value));
        } else {
            BOOST_CHECK_EQUAL(value, test.first);
        }
    }
}

BOOST_AUTO_TEST_CASE(singlefdouble) {
    const std::vector<std::pair<double, std::string>> cases{
        {0.0, std::string("\xfa\x00\x00\x00\x00", 5)},
        {-0.0, std::string("\xfa\x80\x00\x00\x00", 5)},
        {1.0, std::string("\xfa\x3f\x80\x00\x00", 5)},
        {1.1f, std::string("\xfa\x3f\x8c\xcc\xcd", 5)},
        {1.5, std::string("\xfa\x3f\xc0\x00\x00", 5)},
        {65504.0, std::string("\xfa\x47\x7f\xe0\x00", 5)},
        {3.4028234663852886e+38, std::string("\xfa\x7f\x7f\xff\xff", 5)},
        {1.0e+300, std::string("\xfa\x7f\x80\x00\x00", 5)},
        {5.960464477539063e-8, std::string("\xfa\x33\x80\x00\x00", 5)},
        {0.00006103515625, std::string("\xfa\x38\x80\x00\x00", 5)},
        {-4.0, std::string("\xfa\xc0\x80\x00\x00", 5)},
        {-4.1, std::string("\xfa\xc0\x83\x33\x33", 5)},
        {std::numeric_limits<double>::infinity(), std::string("\xfa\x7f\x80\x00\x00", 5)},
        {std::numeric_limits<double>::quiet_NaN(), std::string("\xfa\x7f\xc0\x00\x00", 5)},
        {-std::numeric_limits<double>::infinity(), std::string("\xfa\xff\x80\x00\x00", 5)},
    };
    for (const auto& test : cases) {
        std::string buffer;
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeSingleFloat(buffer, test.first));
        BOOST_CHECK_EQUAL(len, test.second.size());
#if 0
        std::cout << "Encoding: ";
        for (unsigned ch : buffer) {
            std::cout << "\\x" << std::hex << std::setw(2) << std::setfill('0') << (ch&0xFFu);
        }
        std::cout << std::endl;
#endif
        BOOST_CHECK_EQUAL(buffer, test.second);
        float value = 0;
        auto pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeSingleFloat(pos, std::end(test.second), value));
        BOOST_CHECK(pos == std::end(test.second));
        BOOST_CHECK_EQUAL(len, test.second.size());
        if (std::isnan(test.first)) {
            BOOST_CHECK(std::isnan(value));
        } else {
            BOOST_CHECK_EQUAL(value, static_cast<float>(test.first));
        }
    }
}

BOOST_AUTO_TEST_CASE(doublef) {
    const std::vector<std::pair<double, std::string>> cases{
        {0.0, std::string("\xfb\x00\x00\x00\x00\x00\x00\x00\x00", 9)},
        {-0.0, std::string("\xfb\x80\x00\x00\x00\x00\x00\x00\x00", 9)},
        {1.0, std::string("\xfb\x3f\xf0\x00\x00\x00\x00\x00\x00", 9)},
        {1.1, std::string("\xfb\x3f\xf1\x99\x99\x99\x99\x99\x9a", 9)},
        {1.5, std::string("\xfb\x3f\xf8\x00\x00\x00\x00\x00\x00", 9)},
        {65504.0, std::string("\xfb\x40\xef\xfc\x00\x00\x00\x00\x00", 9)},
        {3.4028234663852886e+38, std::string("\xfb\x47\xef\xff\xff\xe0\x00\x00\x00", 9)},
        {1.0e+300, std::string("\xfb\x7e\x37\xe4\x3c\x88\x00\x75\x9c", 9)},
        {5.960464477539063e-8, std::string("\xfb\x3e\x70\x00\x00\x00\x00\x00\x00", 9)},
        {0.00006103515625, std::string("\xfb\x3f\x10\x00\x00\x00\x00\x00\x00", 9)},
        {-4.0, std::string("\xfb\xc0\x10\x00\x00\x00\x00\x00\x00", 9)},
        {-4.1, std::string("\xfb\xc0\x10\x66\x66\x66\x66\x66\x66", 9)},
        {std::numeric_limits<double>::infinity(), std::string("\xfb\x7f\xf0\x00\x00\x00\x00\x00\x00", 9)},
        {std::numeric_limits<double>::quiet_NaN(), std::string("\xfb\x7f\xf8\x00\x00\x00\x00\x00\x00", 9)},
        {-std::numeric_limits<double>::infinity(), std::string("\xfb\xff\xf0\x00\x00\x00\x00\x00\x00", 9)},
    };
    for (const auto& test : cases) {
        std::string buffer;
        std::size_t len;
        BOOST_CHECK_NO_THROW(len = CborLite::encodeDoubleFloat(buffer, test.first));
        BOOST_CHECK_EQUAL(len, test.second.size());
#if 0
        std::cout << "Encoding: ";
        for (unsigned ch : buffer) {
            std::cout << "\\x" << std::hex << std::setw(2) << std::setfill('0') << (ch&0xFFu);
        }
        std::cout << std::endl;
#endif
        BOOST_CHECK_EQUAL(buffer, test.second);
        double value = 0;
        auto pos = std::begin(test.second);
        BOOST_CHECK_NO_THROW(len = CborLite::decodeDoubleFloat(pos, std::end(test.second), value));
        BOOST_CHECK(pos == std::end(test.second));
        BOOST_CHECK_EQUAL(len, test.second.size());
        if (std::isnan(test.first)) {
            BOOST_CHECK(std::isnan(value));
        } else {
            BOOST_CHECK_EQUAL(value, test.first);
        }
    }
}

BOOST_AUTO_TEST_SUITE_END()
