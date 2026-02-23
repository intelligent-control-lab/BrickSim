import std;
import bricksim.physx.breakage;
import bricksim.vendor;

using namespace bricksim;

BreakageDebugDump load_dump(const std::string &filepath) {
	std::ifstream ifs(filepath);
	if (!ifs) {
		throw std::runtime_error(
		    std::format("Failed to open input file: {}", filepath));
	}
	nlohmann::ordered_json j;
	ifs >> j;
	BreakageDebugDump dump;
	j.get_to(dump);
	return dump;
}

void print_solution_info(const BreakageSolution &sol) {
	std::println("Solution info: {}", sol.info.to_string());
	std::println("slack_fraction: {:.6e}", sol.slack_fraction);
}

auto now() {
	return std::chrono::high_resolution_clock::now();
}

double elapsed_ms(const auto &start, const auto &end) {
	return std::chrono::duration<double, std::milli>(end - start).count();
}

int main(int argc, char **argv) {
	if (argc != 2) {
		std::cerr << "Usage: solve_breakage <input_json_file>" << std::endl;
		return 1;
	}

	BreakageDebugDump dump = load_dump(argv[1]);
	if (!dump.prev_state) {
		std::cerr << "Input dump does not contain prev_state." << std::endl;
		return 1;
	}

	BreakageChecker checker;
	checker.thresholds = dump.thresholds;

	auto run_solve = [&](const BreakageState &initial_state) {
		BreakageState state = initial_state;
		auto t0 = now();
		BreakageSolution sol = checker.solve(dump.system, dump.input, state);
		auto t1 = now();
		std::println("Time: {:.3f} ms.", elapsed_ms(t0, t1));
		if (sol.x.size() > 0 && sol.x.size() == dump.solution.x.size()) {
			std::println("Max diff from dumped solution: {:.6e}",
			             (sol.x - dump.solution.x).cwiseAbs().maxCoeff());
		}
		print_solution_info(sol);
	};

	std::println("==== With QP state 1st run ====");
	run_solve(*dump.prev_state);
	std::println("==== With QP state 2nd run ====");
	run_solve(*dump.prev_state);

	BreakageState dropped_state = *dump.prev_state;
	dropped_state.clear_solver_state();
	std::println("==== Without QP state 1st run ====");
	run_solve(dropped_state);
	std::println("==== Without QP state 2nd run ====");
	run_solve(dropped_state);

	return 0;
}
