"""Check the public stdout report and real loopback sends; no trace files."""
import os
from pathlib import Path
import re
import socket
import subprocess
import sys
import tempfile
import threading


def rows(output, section, tree=False):
    text = output.split(section + '\n', 1)[1].split('\n\n', 1)[0]
    lines = [line for line in text.splitlines() if line.startswith('|')]
    columns = [column.strip() for column in lines[0].split('|')[1:-1]]
    assert columns[2:7] == ['Avg', 'P50', 'P95', 'P99', 'P999'], columns

    def value(cell):
        if cell.strip() in ('-', 'unavailable'):
            return None
        number, *unit = cell.strip().rstrip('%').split()
        # Independently normalize the human-readable durations to milliseconds.
        return float(number.replace(',', '')) * {'s': 1000, 'ms': 1, 'us': .001, 'ns': .000001}.get(
            unit[0] if unit else '', 1)

    result = {}
    path = []
    for line in lines[1:]:
        parts = line.split('|')[1:-1]
        name = parts[0][1:].rstrip()
        if tree:
            branch = re.fullmatch(r'((?:│  |   )*)(?:├─ |└─ )(.*)', name)
            depth = len(branch[1]) // 3 + 1 if branch else 0
            function = branch[2] if branch else name
            function = re.sub(r'@\d+(?=\])', '', function)
            path = path[:depth] + [function]
            name = '/'.join(path)
        else:
            name = name.strip().split('@')[0]
        result[name] = dict(zip(columns[1:], map(value, parts[1:])))
    return result


def main():
    binary = str(Path(sys.argv[1]).resolve())
    with tempfile.TemporaryDirectory() as directory, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        env = dict(os.environ, PATRONUS_PROFILE_DIR=directory, PATRONUS_TRACE_DIR=directory)

        def run(*args):
            result = subprocess.run([binary, *args], env=env, cwd=directory,
                                    capture_output=True, text=True, timeout=20)
            assert result.returncode == 0, result.stdout + result.stderr
            assert 'CRITICAL' not in result.stderr and 'WARNING' not in result.stderr, result.stderr
            assert '=== End Patronus profiling ===' in result.stdout, result.stdout
            assert not list(Path(directory).iterdir()), 'Profiler created files'
            return result.stdout

        output = run('--summary')
        wall = rows(output, 'Hotspots: inclusive wall')
        tail = wall['test/tail']
        assert [tail[k] for k in ('Calls', 'P50', 'P95', 'P99', 'P999')] == [1000, 1, 70, 70, 300], tail
        assert tail['Avg'] == 27.499, tail
        # Actual stdout is a bordered, aligned table, including long metric names.
        table = output.split('Hotspots: inclusive wall\n', 1)[1].split('\n\n', 1)[0]
        lines = table.splitlines()
        assert lines[0].startswith('+') and lines[-1] == lines[0], table
        assert len({len(line) for line in lines}) == 1, table
        separators = [[i for i, c in enumerate(line) if c == '|']
                      for line in lines if line.startswith('|')]
        assert all(positions == separators[0] for positions in separators), table
        self_wall = rows(output, 'Hotspots: self wall')
        cpu = rows(output, 'Hotspots: inclusive CPU (available measurements only)')
        self_cpu = rows(output, 'Hotspots: self CPU (unavailable measurements omitted)')
        assert self_wall['test/parent']['Total'] == 10, self_wall
        assert self_cpu['test/parent']['Total'] == 20, self_cpu
        assert 'other/child' in self_wall  # nesting spans streams, not stream-local stacks
        assert 'test/unknown' not in cpu and 'test/unknown' not in self_cpu
        assert 'test/crossing' not in self_wall and 'test/crossing' in wall
        residence = rows(output, 'Pipeline residence/capture delivery cadence (matched subset, not GPU kernel time)')
        assert residence['rgb/inference']['Calls'] == 1 and residence['rgb/inference']['P50'] == 5, residence
        assert residence['mono/inference']['P50'] == 9, residence
        assert 'rgb/source_to_encoder' not in residence
        engine = rows(output, 'Motor function tree', tree=True)
        root = 'set_velocity [gimbal-2]'
        assert engine[root]['Total'] == 100, engine  # not sum of nested calls
        assert engine[root + '/set_velocity_locked/write_axis/setTargetVelocity']['Avg'] == 5
        assert engine[root + '/lock_bus']['Avg'] == 10
        assert '├─ lock_bus' in output
        assert '└─ set_velocity_locked' in output
        assert '   └─ write_axis' in output
        for title in ('Inference function tree', 'Motor function tree'):
            tree_lines = output.split(title + '\n', 1)[1].split('\n\n', 1)[0].splitlines()
            assert len({len(line) for line in tree_lines}) == 1, tree_lines
            positions = [[i for i, char in enumerate(line) if char == '|']
                         for line in tree_lines if line.startswith('|')]
            assert all(value == positions[0] for value in positions), tree_lines
        inference = rows(output, 'Inference function tree', tree=True)
        assert inference['osd_sink_pad_buffer_probe [rgb]/draw_predictions/project_prediction']['Avg'] == 20
        assert all('plugin' not in name and 'nvinfer' not in name for name in inference)
        stages = rows(output, 'Stage summary (host timings)')
        assert stages['gimbal-2 motor host operations']['Total'] == 100
        assert stages['rgb inference host latency']['Avg'] is None  # missing endpoints, no child sums

        overflow = run('--overflow')
        assert 'retained_events=1000000' in overflow, overflow
        assert 'dropped_events=0' not in overflow
        assert 'all distributions are partial' in overflow
        assert not rows(overflow, 'Hotspots: self wall')
        assert not rows(overflow, 'Hotspots: self CPU (unavailable measurements omitted)')

        receiver.bind(('127.0.0.1', 0))
        receiver.settimeout(.1)
        packets = []
        stop = threading.Event()

        def receive():
            while not stop.is_set():
                try:
                    packets.append(receiver.recv(65535))
                except socket.timeout:
                    pass

        thread = threading.Thread(target=receive)
        thread.start()
        try:
            output = run(directory, str(receiver.getsockname()[1]))
        finally:
            stop.set()
            thread.join()
        assert sorted(map(len, packets)) == [23] * 3 + [64] * 34, list(map(len, packets))
        assert 'dropped_events=0' in output
        wall = rows(output, 'Hotspots: inclusive wall')
        cpu = rows(output, 'Hotspots: inclusive CPU (available measurements only)')
        for stream in ('gimbal-0', 'gimbal-1'):
            assert wall[f'{stream}/prediction']['Calls'] == 31
            assert cpu[f'{stream}/cpu_work']['Total'] >= 5
            assert cpu[f'{stream}/sleep']['Total'] < wall[f'{stream}/sleep']['Total'] / 2
        assert 'rgb operations=18 failures=0 bytes=1157.000' in output, output
        assert 'mono operations=17 failures=0 bytes=1088.000' in output, output
        resources = rows(output, 'Resources (units in metric names; sampled every 250ms)')
        for metric in ('process_cpu_percent', 'system_cpu_percent', 'process_rss_bytes',
                       'process_virtual_bytes', 'system_memory_used_bytes'):
            assert metric in resources and resources[metric]['Samples'] > 0, metric
        residence = rows(output, 'Pipeline residence/capture delivery cadence (matched subset, not GPU kernel time)')
        for camera in ('rgb', 'mono'):
            assert residence[f'{camera}/inference']['Calls'] == 17
            assert residence[f'{camera}/inference']['P50'] >= .8
        inference = rows(output, 'Inference function tree', tree=True)
        stages = rows(output, 'Stage summary (host timings)')
        for camera in ('rgb', 'mono'):
            root = 'osd_sink_pad_buffer_probe [' + camera + ']'
            assert inference[root]['Calls'] == 17
            assert inference[root + '/draw_predictions/project_prediction']['Calls'] == 17
            assert stages[camera + ' inference host latency']['Calls'] == 17
            assert stages[camera + ' inference host latency']['Avg'] > residence[camera + '/inference']['Avg']
            assert stages[camera + ' post-capture buffer preparation']['Calls'] == 17
        assert 'Motor function tree' not in output
        assert 'No motor functions recorded' in output
        assert stages['motor host operations (not exercised)']['Avg'] is None
        assert 'Frame capture tree' not in output
        assert '│  └─ project_prediction' in output  # a later sibling still follows this branch
        assert all('plugin' not in name and 'convert_and_prepare' not in name for name in inference)
        assert 'source buffer memory: SystemMemory' in output, output
        print('Stdout percentiles, nesting, CPU work/wait, resources, 37 UDP packets; no files: OK')


if __name__ == '__main__':
    main()
