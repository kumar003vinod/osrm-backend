#include <boost/test/floating_point_comparison.hpp>
#include <boost/test/unit_test.hpp>

#include "engine/polyline_compressor.hpp"
#include "util/coordinate_calculation.hpp"

#include <osrm/coordinate.hpp>

#include <cmath>
#include <vector>

BOOST_AUTO_TEST_SUITE(polyline)

using namespace osrm;
using namespace osrm::engine;

using Lng = osrm::util::FloatLongitude;
using Lat = osrm::util::FloatLatitude;

BOOST_AUTO_TEST_CASE(decode)
{
    // Polyline string for the 5 coordinates
    const std::string polyline = "_c`|@_c`|@o}@_pRo}@_pRo}@_pRo}@_pR";
    const auto coords = decodePolyline(polyline);

    // Test coordinates; these would be the coordinates we give the loc parameter,
    // e.g. loc=10.00,10.0&loc=10.01,10.1...
    std::vector<util::Coordinate> cmp_coords = {{Lng{10.0}, Lat{10.00}},
                                                {Lng{10.1}, Lat{10.01}},
                                                {Lng{10.2}, Lat{10.02}},
                                                {Lng{10.3}, Lat{10.03}},
                                                {Lng{10.4}, Lat{10.04}}};

    BOOST_CHECK_EQUAL(cmp_coords.size(), coords.size());

    for (unsigned i = 0; i < cmp_coords.size(); ++i)
    {
        BOOST_CHECK_CLOSE(static_cast<double>(util::toFloating(coords[i].lat)),
                          static_cast<double>(util::toFloating(cmp_coords[i].lat)),
                          0.0001);
        BOOST_CHECK_CLOSE(static_cast<double>(util::toFloating(coords[i].lon)),
                          static_cast<double>(util::toFloating(cmp_coords[i].lon)),
                          0.0001);
    }
}

BOOST_AUTO_TEST_CASE(encode)
{
    // Coordinates; these would be the coordinates we give the loc parameter,
    // e.g. loc=10.00,10.0&loc=10.01,10.1...

    // Test polyline string for the 5 coordinates
    const std::string polyline = "_c`|@_c`|@o}@_pRo}@_pRo}@_pRo}@_pR";

    // Put the test coordinates into the vector for comparison
    std::vector<util::Coordinate> cmp_coords = {{Lng{10.0}, Lat{10.00}},
                                                {Lng{10.1}, Lat{10.01}},
                                                {Lng{10.2}, Lat{10.02}},
                                                {Lng{10.3}, Lat{10.03}},
                                                {Lng{10.4}, Lat{10.04}}};

    const auto encodedPolyline = encodePolyline<100000>(cmp_coords.begin(), cmp_coords.end());

    BOOST_CHECK_EQUAL(encodedPolyline, polyline);
}

BOOST_AUTO_TEST_CASE(encode6)
{
    // Coordinates; these would be the coordinates we give the loc parameter,
    // e.g. loc=10.00,10.0&loc=10.01,10.1...

    // Test polyline string for the 6 coordinates
    const std::string polyline = "_gjaR_gjaR_pR_ibE_pR_ibE_pR_ibE_pR_ibE";

    // Put the test coordinates into the vector for comparison
    std::vector<util::Coordinate> cmp_coords = {{Lng{10.0}, Lat{10.00}},
                                                {Lng{10.1}, Lat{10.01}},
                                                {Lng{10.2}, Lat{10.02}},
                                                {Lng{10.3}, Lat{10.03}},
                                                {Lng{10.4}, Lat{10.04}}};

    const auto encodedPolyline = encodePolyline<1000000>(cmp_coords.begin(), cmp_coords.end());

    BOOST_CHECK_EQUAL(encodedPolyline, polyline);
}

BOOST_AUTO_TEST_CASE(polyline_sign_check)
{
    std::vector<util::Coordinate> coords = {
        {Lng{0}, Lat{0}}, {Lng{-0.00001}, Lat{0.00000}}, {Lng{0.00000}, Lat{-0.00001}}};

    const auto polyline = encodePolyline<100000>(coords.begin(), coords.end());
    const auto result = decodePolyline(polyline);

    BOOST_CHECK(coords.size() == result.size());
    for (unsigned i = 0; i < result.size(); ++i)
    {
        BOOST_CHECK(coords[i] == result[i]);
    }
}

BOOST_AUTO_TEST_CASE(polyline_short_strings)
{
    std::vector<util::Coordinate> coords = {{Lng{13.44521}, Lat{52.53251}},
                                            {Lng{13.39851}, Lat{52.48362}},
                                            {Lng{13.32573}, Lat{52.52165}},
                                            {Lng{13.32476}, Lat{52.52632}},
                                            {Lng{13.30179}, Lat{52.59155}},
                                            {Lng{13.30179}, Lat{52.60391}}};

    const auto polyline = encodePolyline<100000>(coords.begin(), coords.end());
    BOOST_CHECK(polyline.back() == '?');

    const auto result = decodePolyline(polyline);
    BOOST_CHECK(coords.size() == result.size());
    for (unsigned i = 0; i < result.size(); ++i)
    {
        BOOST_CHECK(coords[i] == result[i]);
    }

    const auto result_short = decodePolyline(polyline.substr(0, polyline.size() - 1));
    BOOST_CHECK(coords.size() == result_short.size());
    for (unsigned i = 0; i < result_short.size(); ++i)
    {
        BOOST_CHECK(coords[i] == result_short[i]);
    }
}

BOOST_AUTO_TEST_CASE(incorrect_polylines)
{
    util::Coordinate coord{Lng{0}, Lat{0}};
    std::vector<std::string> polylines = {"?", "_", "?_"};

    for (auto polyline : polylines)
    {
        const auto result = decodePolyline(polyline);
        BOOST_CHECK(result.size() == 1);
        BOOST_CHECK(result.front() == coord);
    }
}

BOOST_AUTO_TEST_SUITE_END()
