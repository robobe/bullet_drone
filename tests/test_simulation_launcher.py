from examples.simulation_launcher.cli import CliParser, Options


def test_launcher_defaults_to_sitl_and_render():
    assert CliParser().parse([]) == Options()


def test_launcher_can_disable_either_process():
    assert CliParser().parse(['--no-sitl']).sitl is False
    assert CliParser().parse(['--no-render']).render is False
    options = CliParser().parse(['--no-sitl', '--no-render'])
    assert not options.sitl and not options.render
