import argparse
import lego_assemble.utils.kit_runner

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "target",
        help=(
            "Module name or script path, optionally with ':func' suffix, "
            "e.g. 'demos.demo_r1lite', 'demos.demo_r1lite:main', "
            "'/abs/path/to/demo_r1lite.py', or '/abs/path/to/demo_r1lite.py:main'."
        ),
    )
    args = parser.parse_args()
    lego_assemble.utils.kit_runner.run(args.target)

if __name__ == "__main__":
    main()
