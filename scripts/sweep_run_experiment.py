import argparse
import pathlib
import subprocess
import random


HDF5_EXTENSION  = ".h5"
EXECUTABLE_NAME = 'RunExperiment'
EXECUTABLE_PATH = None

# Find the project root and binary directory and the executable (regardless of its file extension).
PROJECT_ROOT_PATH = pathlib.Path(__file__).parent.resolve().parent
BINARIES_PATH     = PROJECT_ROOT_PATH / 'bin'
assert(BINARIES_PATH.exists() and 
       BINARIES_PATH.is_dir() and 
      "Cannot find binary directory: " + str(BINARIES_PATH))

for item in BINARIES_PATH.iterdir():
    if item.is_file() and item.name.find(EXECUTABLE_NAME) == 0:
        EXECUTABLE_PATH = item
        break

assert(EXECUTABLE_PATH is not None and 
       EXECUTABLE_PATH.exists() and 
       EXECUTABLE_PATH.is_file() and
       "Cannot find executable in: " + str(BINARIES_PATH))


# Find the GroundTruth directory and all of the HSF5 scene files in it.
GROUND_TRUTH_PATH = PROJECT_ROOT_PATH / "share" / "GroundTruth"
assert(GROUND_TRUTH_PATH.exists() and 
       GROUND_TRUTH_PATH.is_dir() and 
      "Cannot find GroundTruth directory: " + str(GROUND_TRUTH_PATH))

GROUND_TRUTH_FILES: list[pathlib.Path] = []
for item in GROUND_TRUTH_PATH.iterdir():
    if item.is_file() and item.name.endswith(HDF5_EXTENSION):
        GROUND_TRUTH_FILES.append(item)


# File constants
STDIN_NEWLINE = "\n" 


# Exploration space
REGULAR_RERUNS = 1
RANDOM_RERUNS  = 10

VIEW_RADIUS = 2.5
REJECTION_RATE = 0.0

RANDOM_SEED = False

N_VIEWS = [5, 10, 15, 20]
N_REPS  = [1, 2, 3, 4]  # TODO: use this?


# ([Name], [Intrinsic args])
INTRINSICS: list[tuple[str, str]] = [
    (
        "RealSense_d455",
        "--d455 1.0"
    )
]


# ([Name], [Policy arg string], [Number of re-runs])
METHODS: list[tuple[str, str, str]] = [
    (
        "Sphere_Uniform",
        "--type sphere --uniform --r " + str(VIEW_RADIUS),
        REGULAR_RERUNS
    ),
    (
        "Sphere_Unordered",
        "--type sphere --uniform --unordered --r " + str(VIEW_RADIUS),
        REGULAR_RERUNS
    ),
    (
        "Sphere_Random",
        "--type sphere --r " + str(VIEW_RADIUS),
        RANDOM_RERUNS
    ),
    (
        "Axis_Z-axis",
        "--type axis --z-axis --r " + str(VIEW_RADIUS),
        REGULAR_RERUNS
    ),
    (
        "Axis_Random",
        "--type axis --random-axis --r " + str(VIEW_RADIUS),
        RANDOM_RERUNS
    ),
]


def call_process(fpath: pathlib.Path, scene: pathlib.Path, intr: str,
                 policy: str, n_views: int, seed: int = 0, parsed_args: argparse.Namespace = None):
    stdin  = ""
    stdin += str(fpath) + STDIN_NEWLINE
    stdin += str(scene) + STDIN_NEWLINE
    stdin += str(REJECTION_RATE) + STDIN_NEWLINE
    stdin += intr + STDIN_NEWLINE
    stdin += policy + " --n-views " + str(n_views) + " --seed " + str(seed) + STDIN_NEWLINE

    # Add data channels
    stdin += "--name probability  --type Probability  --dtype float" + STDIN_NEWLINE
    stdin += "--name TSDF         --type TSDF         --dtype float" + STDIN_NEWLINE
    stdin += "--name binary       --type binary" + STDIN_NEWLINE

    # Add metrics
    stdin += STDIN_NEWLINE

    proc = subprocess.Popen(executable=EXECUTABLE_PATH, args=[], text=True,
                            stdin=subprocess.PIPE, stdout=subprocess.PIPE)
    stdout = proc.communicate(stdin)
    if parsed_args is not None and parsed_args.pause:
        print(stdin)
        print(stdout[0])
        input("End of process...")


def main(parsed_args: argparse.Namespace) -> None:
    """
    Program entry point.
    """
    start = parsed_args.start_at
    n = 0
    N = len(INTRINSICS) * len(GROUND_TRUTH_FILES) * \
        len(N_VIEWS) * sum([m[2] for m in METHODS])
    print(f"Generating {N} experiments, beginning at experiment {start}...")

    fpath_base = PROJECT_ROOT_PATH / "share" / "Results"
    for intr in INTRINSICS:
        for scene in GROUND_TRUTH_FILES:
            for policy in METHODS:
                for n_views in N_VIEWS:
                    for rep in range(1, policy[2] + 1):
                        n += 1
                        if (n < start):
                            continue
                        seed = random.randrange(1, 1e8) if RANDOM_SEED else rep

                        fpath = pathlib.Path(intr[0], scene.name.removesuffix(HDF5_EXTENSION),
                                             policy[0], str(n_views), str(rep))
                        print(f"({n} / {N}) {fpath}")
                        fpath = fpath_base / fpath
                        if fpath.exists() is False:
                            fpath.mkdir(parents=True)
                        fpath /= "results" + HDF5_EXTENSION
                        call_process(fpath, scene, intr[1], policy[1], n_views, seed, parsed_args)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='Sweep Run Experiment',
        description='This script runs the Run Experiment script for a wide sweep of parameters.'
    )
    parser.add_argument(
        "-p", "--pause",
        action="store_true",
        default=False,
        help="Optionally pause after each experiment and display stdin/stdout."
    )
    parser.add_argument(
        "--start-at",
        type=int,
        default=0,
        help="Start some number into the sweep."
    )

    args = parser.parse_args()
    try:
        main(args)
    except KeyboardInterrupt as e:
        print("\nExiting early on KeyboardInterrupt.")
