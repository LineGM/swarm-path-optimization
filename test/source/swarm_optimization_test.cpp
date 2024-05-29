#include "PSO.hpp"
#include <boost/ut.hpp>

auto main() -> int
{

	using namespace boost::ut;

	"Trivial"_test = [] {
		expect(1 == 1) << "1"
					   << "!="
					   << "1";
	};
}