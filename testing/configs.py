from ralph.test_entry import TestEntry
from ralph.config_utils import check_valid
from ralph.generators.simple import generate_simple_exec


TestEntry.set_default_whisper_path("build-Linux/whisper")
#TestEntry.set_default_whisper_config_path("configuration/bh-tt-whisper.json")

def generate_dcache_tests():
    rss_bench = "ubench/rss.c"

    def rss_generator(rss: int):
        compile_flags = f"-DRSS={rss}  -mcmodel=medany -static -std=gnu99 -O2 -nostdlib  -fno-tree-loop-distribute-patterns -DPREALLOCATE=1 -march=rv64imafdc -mabi=lp64d"
        return TestEntry(name=f"rss_{rss}",
                         path_to_benchmark=rss_bench,
                         compile_flags=compile_flags,
                         whisper_flags="--isa rv64imafdc --newlib",
                         description=f"8-byte strided pointer chase across {rss} bytes; cache capacity test",
                         whisper_cycles=300000000)

    rss_tests = [rss_generator(rss) for rss in [32, 64, 128, 256, 512, 1024]]
    return rss_tests


def get_microprobe_name(path: str) -> str:
    return path.split('/')[-1].split('.')[0]


def n_hop_tests(instruction="ADD_V0", desc="{dep_distance}-hop test"):
    tests = []

    for dep_distance in range(1, 5):
        test_name = f"{instruction}_{dep_distance}_hop"
        # We generate a test with microprobe
        test_path = generate_simple_exec(
            name=f"{test_name}", instructions=instruction, dep_distance=dep_distance)

        # A test entry is created
        test = TestEntry.microprobe_test(name=test_name,
                                         path_to_benchmark=test_path,
                                         whisper_cycles=10000000)
        tests.append(test)
    return tests


def issue_tests():
    return n_hop_tests()


def memory_tests():
    return n_hop_tests(instruction="LD_V0", desc="{dep_distance}-hop test with ld dw")


all_tests = dict(dcache=generate_dcache_tests(), n_hop_tests=issue_tests(),
                 memory_tests=memory_tests())


if __name__ == "__main__":
    check_valid(all_tests, "trace")
