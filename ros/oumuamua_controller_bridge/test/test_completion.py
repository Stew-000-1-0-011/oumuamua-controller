from oumuamua_controller_bridge.completion import complete


def test_complete_command():
    start, cands = complete("ech", 3, "/tmp")
    assert start == 0
    assert "echo" in cands


def test_complete_file(tmp_path):
    (tmp_path / "alpha.txt").write_text("x")
    (tmp_path / "alps").mkdir()
    (tmp_path / "zzz").write_text("x")
    start, cands = complete("cat al", 6, str(tmp_path))
    assert start == 4
    assert "alpha.txt" in cands and "alps/" in cands
    assert "zzz" not in cands
    assert cands == sorted(set(cands))


def test_complete_respects_cwd(tmp_path):
    a = tmp_path / "a"
    b = tmp_path / "b"
    a.mkdir()
    b.mkdir()
    (a / "only_in_a.txt").write_text("x")
    _, ca = complete("ls only", 7, str(a))
    _, cb = complete("ls only", 7, str(b))
    assert ca == ["only_in_a.txt"]
    assert cb == []


def test_complete_setup_files_and_cursor_mid_line(tmp_path):
    setup = tmp_path / "setup.sh"
    setup.write_text("my_special_cmd() { :; }\n")
    start, cands = complete("my_spec foo", 7, str(tmp_path), [str(setup)])
    assert start == 0
    assert "my_special_cmd" in cands
