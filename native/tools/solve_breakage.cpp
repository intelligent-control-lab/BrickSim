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
	std::unique_ptr<BreakageState> state;
	if (dump.prev_state.has_value()) {
		state = std::make_unique<BreakageState>(*dump.prev_state);
	} else {
		state = std::make_unique<BreakageState>(dump.state);
		std::println("Warning: no previous state provided in debug dump.");
	}

	auto t0 = std::chrono::high_resolution_clock::now();

	BreakageSolution sol = checker.solve(dump.system, dump.input, *state);

	auto t1 = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> elapsed = t1 - t0;
	std::println("Took {:.3f} ms.", elapsed.count() * 1e3);

	double max_deviation = (sol.x - dump.solution.x).cwiseAbs().maxCoeff();
	std::println("Max deviation: {:.6e}", max_deviation);

	return 0;
}
