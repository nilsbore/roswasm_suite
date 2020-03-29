// This file is part of CBOR-lite which is copyright Isode Limited
// and others and released under a MIT license. For details, see the
// COPYRIGHT.md file in the top-level folder of the CBOR-lite software
// distribution.
#include <cbor-lite/codec-fp.h>
#include <boost/test/unit_test.hpp>
#include <tuple>

BOOST_AUTO_TEST_SUITE(cbor_lite)
BOOST_AUTO_TEST_SUITE(floating_point)

BOOST_AUTO_TEST_CASE(singlef) {
    const std::vector<std::pair<float, const std::string>> cases {
        {0.0f, std::string("\xfa\x00\x00\x00\x00", 5)}, {-0.0f, std::string("\xfa\x80\x00\x00\x00", 5)},
            {1.0f, std::string("\xfa\x3f\x80\x00\x00", 5)}, {1.1f, std::string("\xfa\x3f\x8c\xcc\xcd", 5)},
            {1.5f, std::string("\xfa\x3f\xc0\x00\x00", 5)}, {65504.0f, std::string("\xfa\x47\x7f\xe0\x00", 5)},
            {3.4028234663852886e+38f, std::string("\xfa\x7f\x7f\xff\xff", 5)},
#if 0 // too large for single
            {1.0e+300f, std::string("\xfa\x7f\x80\x00\x00", 5)},
#endif
            {5.960464477539063e-8f, std::string("\xfa\x33\x80\x00\x00", 5)},
            {0.00006103515625f, std::string("\xfa\x38\x80\x00\x00", 5)}, {-4.0f, std::string("\xfa\xc0\x80\x00\x00", 5)},
            {-4.1f, std::string("\xfa\xc0\x83\x33\x33", 5)},
            {std::numeric_limits<float>::infinity(), std::string("\xfa\x7f\x80\x00\x00", 5)},
            {std::numeric_limits<float>::quiet_NaN(), std::string("\xfa\x7f\xc0\x00\x00", 5)},
            {-std::numeric_limits<float>::infinity(), std::string("\xfa\xff\x80\x00\x00", 5)},
    };
    for (const auto& test : cases) {
        BOOST_CHECK_NO_THROW({
            std::string buffer;
            auto len = CborLite::encodeSingleFloat(buffer, test.first);
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(buffer, test.second);
            float value = 0;
            auto pos = std::begin(test.second);
            len = CborLite::decodeSingleFloat(pos, std::end(test.second), value);
            BOOST_CHECK(pos == std::end(test.second));
            BOOST_CHECK_EQUAL(len, test.second.size());
            if (std::isnan(test.first)) {
                BOOST_CHECK(std::isnan(value));
            } else {
                BOOST_CHECK_EQUAL(value, test.first);
            }
        });
    }
}

BOOST_AUTO_TEST_CASE(singlefdouble) {
    const std::vector<std::pair<double, const std::string>> cases{
        {0.0, std::string("\xfa\x00\x00\x00\x00", 5)},
        {-0.0, std::string("\xfa\x80\x00\x00\x00", 5)},
        {1.0, std::string("\xfa\x3f\x80\x00\x00", 5)},
        {1.1f, std::string("\xfa\x3f\x8c\xcc\xcd", 5)},
        {1.5, std::string("\xfa\x3f\xc0\x00\x00", 5)},
        {65504.0, std::string("\xfa\x47\x7f\xe0\x00", 5)},
        {3.4028234663852886e+38, std::string("\xfa\x7f\x7f\xff\xff", 5)},
        {1.0e+300, std::string("\xfa\x7f\x80\x00\x00", 5)}, // encodes as infinity
        {5.960464477539063e-8, std::string("\xfa\x33\x80\x00\x00", 5)},
        {0.00006103515625, std::string("\xfa\x38\x80\x00\x00", 5)},
        {-4.0, std::string("\xfa\xc0\x80\x00\x00", 5)},
        {-4.1, std::string("\xfa\xc0\x83\x33\x33", 5)},
        {std::numeric_limits<double>::infinity(), std::string("\xfa\x7f\x80\x00\x00", 5)},
        {std::numeric_limits<double>::quiet_NaN(), std::string("\xfa\x7f\xc0\x00\x00", 5)},
        {-std::numeric_limits<double>::infinity(), std::string("\xfa\xff\x80\x00\x00", 5)},
    };
    for (const auto& test : cases) {
        BOOST_CHECK_NO_THROW({
            std::string buffer;
            auto len = CborLite::encodeSingleFloat(buffer, test.first);
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(buffer, test.second);
            float value = 0;
            auto pos = std::begin(test.second);
            len = CborLite::decodeSingleFloat(pos, std::end(test.second), value);
            BOOST_CHECK(pos == std::end(test.second));
            BOOST_CHECK_EQUAL(len, test.second.size());
            if (std::isnan(test.first)) {
                BOOST_CHECK(std::isnan(value));
            } else {
                BOOST_CHECK_EQUAL(value, static_cast<float>(test.first));
            }
        });
    }
}

BOOST_AUTO_TEST_CASE(doublef) {
    const std::vector<std::pair<double, const std::string>> cases{
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
        BOOST_CHECK_NO_THROW({
            std::string buffer;
            auto len = CborLite::encodeDoubleFloat(buffer, test.first);
            BOOST_CHECK_EQUAL(len, test.second.size());
            BOOST_CHECK_EQUAL(buffer, test.second);
            double value = 0;
            auto pos = std::begin(test.second);
            len = CborLite::decodeDoubleFloat(pos, std::end(test.second), value);
            BOOST_CHECK(pos == std::end(test.second));
            BOOST_CHECK_EQUAL(len, test.second.size());
            if (std::isnan(test.first)) {
                BOOST_CHECK(std::isnan(value));
            } else {
                BOOST_CHECK_EQUAL(value, test.first);
            }
        });
    }
}

BOOST_AUTO_TEST_SUITE_END()
BOOST_AUTO_TEST_SUITE_END()
