import std;
import bricksim.core.specs;
import bricksim.core.graph;
import bricksim.core.connections;
import bricksim.io.topology;
import bricksim.physx.breakage;
import bricksim.usd.author;
import bricksim.usd.parse;
import bricksim.utils.type_list;
import bricksim.utils.transforms;
import bricksim.vendor;

template <typename... Args>
static void eprintln(std::format_string<Args...> fmt, Args &&...args) {
	std::println(std::cerr, fmt, std::forward<Args>(args)...);
}

using namespace bricksim;

using Parts = PartList<BrickPart>;
using PartAuthors = type_list<PrototypeBrickAuthor>;
using PartParsers = type_list<BrickParser>;
using Serializer = TopologySerializer<BrickSerializer>;
using Graph = LegoGraph<Parts>;

std::tuple<Graph, ImportedMapping> load_lego_graph(const std::string &path) {
	nlohmann::ordered_json j;
	if (path == "-") {
		std::cin >> j;
	} else {
		std::ifstream ifs(path);
		if (!ifs) {
			throw std::runtime_error(
			    std::format("Failed to open input file '{}'", path));
		}
		ifs >> j;
	}
	JsonTopology topology = j.get<JsonTopology>();
	Graph g;
	ImportedMapping mapping = Serializer{}.import_graph(topology, g);
	return {std::move(g), std::move(mapping)};
}

int main(int argc, char **argv) {
	if (argc != 2) {
		eprintln("Usage: static_solve <input_lego_json>");
		return 1;
	}
	auto [graph, mapping] = load_lego_graph(argv[1]);
	eprintln("Loaded LegoGraph with {} parts and {} connections.",
	         graph.parts().size(), graph.connection_segments().size());

	if (!mapping.parts.contains(0)) {
		eprintln("No part with id 0 found in the mapping.");
		return 1;
	}
	PartId rep_part = mapping.parts.at(0);
	BrickPart rep_brick = graph.parts()
	                          .value_of<SimplePartWrapper<BrickPart>>(rep_part)
	                          .wrapped();
	eprintln("Root part 0 is of size {}x{}x{}.", rep_brick.L(), rep_brick.W(),
	         rep_brick.H());
	int total_cc_count = 0;
	for (auto _ : graph.components()) {
		total_cc_count++;
	}
	auto cc_rep = graph.component_view(rep_part);
	eprintln("Total CCs in the graph: {}. Root CC has {} parts.",
	         total_cc_count, cc_rep.size());

	BreakageChecker checker;
	BreakageThresholds &thr = checker.thresholds;
	auto thr_env = [](const std::string &env_name, double &val) {
		const char *env_val = std::getenv(env_name.c_str());
		if (env_val) {
			try {
				val = std::stod(env_val);
				eprintln("Using {}={}", env_name, val);
			} catch (const std::exception &e) {
				eprintln("Failed to parse {}='{}': {}", env_name, env_val,
				         e.what());
			}
		}
	};
	thr_env("BREAKAGE_CONTACT_REGULARIZATION", thr.ContactRegularization);
	thr_env("BREAKAGE_CLUTCH_AXIAL_COMPLIANCE", thr.ClutchAxialCompliance);
	thr_env("BREAKAGE_CLUTCH_RADIAL_COMPLIANCE", thr.ClutchRadialCompliance);
	thr_env("BREAKAGE_CLUTCH_TANGENTIAL_COMPLIANCE",
	        thr.ClutchTangentialCompliance);
	thr_env("BREAKAGE_FRICTION_COEFFICIENT", thr.FrictionCoefficient);
	thr_env("BREAKAGE_PRELOADED_FORCE", thr.PreloadedForce);
	thr_env("BREAKAGE_SLACK_FRACTION_WARN", thr.SlackFractionWarn);
	thr_env("BREAKAGE_SLACK_FRACTION_B_FLOOR", thr.SlackFractionBFloor);

	BreakageSystem sys = checker.build_system(graph, rep_part);
	eprintln("System has {} contact vertices and {} clutches.",
	         sys.num_contact_vertices(), sys.num_clutches());

	eprintln("==== Connections ====");
	for (typename Graph::ConnSegConstEntry cs_entry :
	     cc_rep.connection_segments()) {
		ConnSegId csid = cs_entry.template key<ConnSegId>();
		const auto &[stud_ifref, hole_ifref] =
		    cs_entry.template key<ConnSegRef>();
		const auto &[stud_pid, stud_ifid] = stud_ifref;
		const auto &[hole_pid, hole_ifid] = hole_ifref;
		InterfaceSpec stud_spec = graph.interface_spec_at(stud_ifref);
		InterfaceSpec hole_spec = graph.interface_spec_at(hole_ifref);
		const ConnectionSegment &cs = cs_entry.value().wrapped();
		ConnectionOverlap overlap = cs.compute_overlap(stud_spec, hole_spec);
		eprintln("Conn #{}: Part #{}/{} ({}x{}) -> Part #{}/{} ({}x{}), "
		         "overlap {}x{}, yaw {}",
		         csid, stud_pid, stud_ifid, stud_spec.L, stud_spec.W, hole_pid,
		         hole_ifid, hole_spec.L, hole_spec.W, overlap.overlap.x(),
		         overlap.overlap.y(), static_cast<int>(cs.yaw));
	}
	eprintln("=====");

	double gravity = 9.8;
	double dt = 1.0 / 60.0;
	int num_parts = sys.num_parts();
	BreakageInput in;
	in.w.resize(num_parts, 3);
	in.v.resize(num_parts, 3);
	in.q.resize(num_parts, 4);
	in.c.resize(num_parts, 3);
	in.J.resize(num_parts, 3);
	in.H.resize(num_parts, 3);
	in.dt = dt;
	double unbalanced_force = 0.0;
	Eigen::Vector3d unbalanced_torque = Eigen::Vector3d::Zero();
	Eigen::Vector3d root_com =
	    graph.parts().visit(rep_part, [&](const auto &pw) {
		    const auto &part = pw.wrapped();
		    return part.com();
	    });
	for (int index = 0; index < num_parts; ++index) {
		PartId pid = sys.part_ids().at(index);
		double mass;
		Eigen::Vector3d com;
		graph.parts().visit(pid, [&](const auto &pw) {
			const auto &part = pw.wrapped();
			mass = part.mass();
			com = part.com();
		});
		Transformd T_root_part = graph.lookup_transform(rep_part, pid).value();
		const auto &[q_root_part, t_root_part] = T_root_part;
		in.w.row(index).setZero();
		in.v.row(index).setZero();
		in.q.row(index) = q_root_part.normalized().coeffs();
		in.c.row(index) = t_root_part + q_root_part * com;
		if (pid != rep_part) {
			in.J.row(index) = Eigen::Vector3d(0.0, 0.0, -mass * gravity * dt);
			in.H.row(index).setZero();
			unbalanced_force += -mass * gravity;
			unbalanced_torque +=
			    (t_root_part + q_root_part * com - root_com)
			        .cross(Eigen::Vector3d(0.0, 0.0, -mass * gravity));
		}
	}
	eprintln("Unbalanced force: {:.6e} N", unbalanced_force);
	eprintln("Unbalanced torque: [{:.6e}, {:.6e}, {:.6e}] Nm",
	         unbalanced_torque.x(), unbalanced_torque.y(),
	         unbalanced_torque.z());
	int row_root = sys.part_id_to_index().at(rep_part);
	in.J.row(row_root) = Eigen::Vector3d{0, 0, -unbalanced_force * dt};
	in.H.row(row_root) = -unbalanced_torque * dt;

	BreakageState state = checker.build_initial_state(sys, in);
	checker.thresholds.DebugDump = true;
	checker.set_debug_dump_dir(std::filesystem::temp_directory_path().string());
	BreakageSolution sol = checker.solve(sys, in, state);
	checker.thresholds.DebugDump = false;
	eprintln("Solution info: {}", sol.info.to_string());
	eprintln("slack_fraction: {:.6e}", sol.slack_fraction);

	BreakageState state2 = checker.build_initial_state(sys, in);
	auto t0 = std::chrono::high_resolution_clock::now();
	BreakageSolution sol2 = checker.solve(sys, in, state2);
	auto t1 = std::chrono::high_resolution_clock::now();
	double elapsed_s = std::chrono::duration<double>(t1 - t0).count();
	eprintln("Solve time: {:.3f} ms.", elapsed_s * 1000);

	bool stable;
	bool solved;
	if (total_cc_count == 1) {
		if (sol.utilization.size() > 0) {
			stable = (sol.utilization.array() <= 1.0).all();
			solved = true;
		} else {
			stable = false;
			solved = false;
		}
	} else {
		stable = false;
		solved = true;
	}

	nlohmann::ordered_json result;
	result["stable"] = stable;
	result["solved"] = solved;
	result["time_s"] = elapsed_s;
	result["cc_count"] = total_cc_count;
	result["slack_fraction"] = sol.slack_fraction;
	result["solution_info"] = sol.info;
	nlohmann::ordered_json utilizations;
	if (sol.utilization.size() > 0) {
		for (int k = 0; k < sys.num_clutches(); ++k) {
			ConnSegId csid = sys.clutch_ids().at(k);
			double utilization = sol.utilization(k);
			utilizations[std::to_string(csid)] = utilization;
		}
	}
	result["clutch_utilizations"] = utilizations;
	std::cout << result.dump(2) << std::endl;

	return 0;
}
