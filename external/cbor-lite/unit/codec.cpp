// This file is part of CBOR-lite which is copyright Isode Limited
// and others and released under a MIT license. For details, see the
// COPYRIGHT.md file in the top-level folder of the CBOR-lite software
// distribution.
#include "../include/cbor-lite/codec.h"
#include <boost/test/unit_test.hpp>
#include <tuple>

BOOST_AUTO_TEST_SUITE(cbor_lite)
BOOST_AUTO_TEST_SUITE(codec)

BOOST_AUTO_TEST_CASE(non_negative) {
    const std::vector<std::pair<std::uint_fast64_t, const std::string>> cases{
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
        BOOST_CHECK_NO_THROW({
            std::string buffer;
            auto len = CborLite::encodeUnsigned(buffer, test.first);
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(buffer, test.second);
            std::uint_fast64_t value = 0u;
            auto pos = std::begin(test.second);
            len = CborLite::decodeUnsigned(pos, std::end(test.second), value);
            BOOST_CHECK(pos == std::end(test.second));
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(value, test.first);
        });
    }
}

BOOST_AUTO_TEST_CASE(negative) {
    const std::vector<std::pair<std::uint_fast64_t, const std::string>> cases{
        {0, "\x20"},
        {9, "\x29"},
        {99, "\x38\x63"},
        {999, "\x39\x03\xe7"},
    };
    for (const auto& test : cases) {
        BOOST_CHECK_NO_THROW({
            std::string buffer;
            auto len = CborLite::encodeNegative(buffer, test.first);
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(buffer, test.second);
            std::uint_fast64_t value = 0u;
            auto pos = std::begin(test.second);
            len = CborLite::decodeNegative(pos, std::end(test.second), value);
            BOOST_CHECK(pos == std::end(test.second));
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(value, test.first);
        });
    }
}

BOOST_AUTO_TEST_CASE(integer) {
    const std::vector<std::pair<std::int_fast64_t, const std::string>> cases{
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
        BOOST_CHECK_NO_THROW({
            std::string buffer;
            auto len = CborLite::encodeInteger(buffer, test.first);
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(buffer, test.second);
            std::int_fast64_t value = 0;
            auto pos = std::begin(test.second);
            len = CborLite::decodeInteger(pos, std::end(test.second), value);
            BOOST_CHECK(pos == std::end(test.second));
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(value, test.first);
        });
    }
}

BOOST_AUTO_TEST_CASE(boolean) {
    const std::vector<std::pair<bool, const std::string>> cases{
        {false, "\xf4"},
        {true, "\xf5"},
    };
    for (const auto& test : cases) {
        BOOST_CHECK_NO_THROW({
            std::string buffer;
            auto len = CborLite::encodeBool(buffer, test.first);
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(buffer, test.second);
            auto value = false;
            auto pos = std::begin(test.second);
            len = CborLite::decodeBool(pos, std::end(test.second), value);
            BOOST_CHECK(pos == std::end(test.second));
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(value, test.first);
        });
    }
}

BOOST_AUTO_TEST_CASE(bytes) {
    const std::vector<std::pair<const std::string, const std::string>> cases{
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
        BOOST_CHECK_NO_THROW({
            std::string buffer;
            auto len = CborLite::encodeBytes(buffer, test.first);
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(buffer, test.second);
            std::string value;
            auto pos = std::begin(test.second);
            len = CborLite::decodeBytes(pos, std::end(test.second), value);
            BOOST_CHECK(pos == std::end(test.second));
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(value, test.first);
        });
    }
    BOOST_CHECK_NO_THROW({
        std::vector<char> buffer;
        std::string input = "@@@@";
        std::vector<char> payload(std::begin(input), std::end(input));
        auto len = CborLite::encodeBytes(buffer, payload);
        std::string expect = "\x44\x40\x40\x40\x40";
        BOOST_CHECK_EQUAL(len, expect.length());
        BOOST_CHECK_EQUAL(std::string(std::begin(buffer), std::end(buffer)), expect);
    });
}

BOOST_AUTO_TEST_CASE(encodedBytes) {
    const std::vector<std::pair<const std::string, const std::string>> cases{
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
        BOOST_CHECK_NO_THROW({
            std::string buffer;
            auto len = CborLite::encodeEncodedBytes(buffer, test.first);
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(buffer, test.second);

            buffer.clear();
            len = CborLite::encodeEncodedBytesPrefix(buffer, test.first.length());
            BOOST_CHECK_EQUAL(len, 3);
            BOOST_CHECK_EQUAL(buffer, test.second.substr(0, 3));

            std::string value;
            auto pos = std::begin(test.second);
            len = CborLite::decodeEncodedBytes(pos, std::end(test.second), value);
            BOOST_CHECK(pos == std::end(test.second));
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(value, test.first);

            std::size_t got = 0u;
            pos = std::begin(test.second);
            len = CborLite::decodeEncodedBytesPrefix(pos, pos + 3, got);
            BOOST_CHECK_EQUAL(len, 3);
            BOOST_CHECK_EQUAL(got, test.first.length());
        });
    }
    BOOST_CHECK_NO_THROW({
        std::vector<char> buffer;
        std::string input = "@@@@";
        std::vector<char> payload(std::begin(input), std::end(input));
        auto len = CborLite::encodeEncodedBytes(buffer, payload);
        std::string expect = "\xd8\x18\x44\x40\x40\x40\x40";
        BOOST_CHECK_EQUAL(len, expect.length());
        BOOST_CHECK_EQUAL(std::string(std::begin(buffer), std::end(buffer)), expect);
    });
}

BOOST_AUTO_TEST_CASE(strings) {
    const std::vector<std::pair<const std::string, const std::string>> cases{
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
        BOOST_CHECK_NO_THROW({
            std::string buffer;
            auto len = CborLite::encodeText(buffer, test.first);
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(buffer, test.second);
            std::string value;
            auto pos = std::begin(test.second);
            len = CborLite::decodeText(pos, std::end(test.second), value);
            BOOST_CHECK(pos == std::end(test.second));
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(value, test.first);
        });
    }
}

BOOST_AUTO_TEST_CASE(array) {
    const std::vector<std::pair<std::uint_fast64_t, const std::string>> cases{
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
        BOOST_CHECK_NO_THROW({
            std::string buffer;
            auto len = CborLite::encodeArraySize(buffer, test.first);
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(buffer, test.second);
            unsigned long long value = 0;
            auto pos = std::begin(test.second);
            len = CborLite::decodeArraySize(pos, std::end(test.second), value);
            BOOST_CHECK(pos == std::end(test.second));
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(value, test.first);
        });
    }
}

BOOST_AUTO_TEST_CASE(map) {
    const std::vector<std::pair<std::uint_fast64_t, const std::string>> cases{
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
        BOOST_CHECK_NO_THROW({
            std::string buffer;
            auto len = CborLite::encodeMapSize(buffer, test.first);
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(buffer, test.second);
            unsigned long long value = 0;
            auto pos = std::begin(test.second);
            len = CborLite::decodeMapSize(pos, std::end(test.second), value);
            BOOST_CHECK(pos == std::end(test.second));
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(value, test.first);
        });
    }
}

BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE_END()
