import std;
import lego_assemble.physx.breakage;
import lego_assemble.vendor;

using namespace lego_assemble;

int main(int argc, char **argv) {
	if (argc != 2) {
		std::cerr << "Usage: solve_breakage <input_json_file>" << std::endl;
		return 1;
	}

	BreakageDebugDump dump;
	{
		std::ifstream ifs(argv[1]);
		if (!ifs) {
			std::cerr << "Failed to open input file: " << argv[1] << std::endl;
			return 1;
		}
		nlohmann::ordered_json j;
		ifs >> j;
		j.get_to(dump);
	}

	BreakageChecker checker;
	checker.thresholds = dump.thresholds;
	BreakageSolution sol = checker.solve(dump.system, dump.input, dump.state);
	return 0;
}
