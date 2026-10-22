from oumuamua_controller_bridge.task_memory import parse


def test_parse_task_memory():
    t = ("holding: gripper0=yes gripper1=no\n"
         "picked blocks: 0 3\n"
         "placed cells (column, row): (0, 1) (2, 3)")
    assert parse(t) == ([True, False], [0, 3], [(0, 1), (2, 3)])


def test_parse_empty():
    t = "holding: gripper0=no gripper1=no\npicked blocks:\nplaced cells (column, row):"
    assert parse(t) == ([False, False], [], [])


def test_parse_garbage_is_empty():
    assert parse("") == ([False, False], [], [])
