from oumuamua_controller_bridge.task_memory import parse


def test_parse_task_memory():
    t = "holding: gripper0=yes gripper1=no\npicked blocks: 0 3"
    assert parse(t) == ([True, False], [0, 3])


def test_absent_placed_cells_line_is_normal():
    # task_commander's ~/state has no "placed cells" line any more
    t = "holding: gripper0=no gripper1=yes\npicked blocks: 4"
    assert parse(t) == ([False, True], [4])


def test_old_placed_cells_line_is_ignored():
    t = ("holding: gripper0=yes gripper1=no\npicked blocks: 0 3\n"
         "placed cells (column, row): (0, 1) (2, 3)")
    assert parse(t) == ([True, False], [0, 3])


def test_parse_empty():
    assert parse("holding: gripper0=no gripper1=no\npicked blocks:") == ([False, False], [])


def test_parse_garbage_is_empty():
    assert parse("") == ([False, False], [])
