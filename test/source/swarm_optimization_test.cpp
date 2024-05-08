#include <boost/ut.hpp>
#include "lib.hpp"

auto main() -> int
{

  using namespace boost::ut;

	"Lib constructor checking"_test = [] {
		expect(nothrow([] {
			auto const lib = library {};
			expect(lib.name == "swarm_optimization") << lib.name << "!=" << "swarm_optimization";
		}));
	};
}