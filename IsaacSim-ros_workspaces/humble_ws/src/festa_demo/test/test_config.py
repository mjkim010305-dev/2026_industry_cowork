import os
import sys
import xml.etree.ElementTree as ET

import pytest
import yaml

from conftest import PKG_DIR

PARAMS = os.path.join(PKG_DIR, 'params', 'festa_demo_nav2.yaml')
BT = os.path.join(PKG_DIR, 'bt', 'festa_demo.xml')


@pytest.fixture(scope='module')
def params():
    with open(PARAMS) as f:
        return yaml.safe_load(f)


def costmaps(params):
    return {name: params[name][name]['ros__parameters'] for name in ('local_costmap', 'global_costmap')}


@pytest.mark.parametrize('name', ['local_costmap', 'global_costmap'])
def test_costmap_margins(params, name):
    cm = costmaps(params)[name]
    infl = cm['inflation_layer']
    assert cm['robot_radius'] == 0.20
    assert infl['inflation_radius'] == 0.20
    assert infl['cost_scaling_factor'] == 5.0
    assert infl['inflation_radius'] >= cm['robot_radius']


@pytest.mark.parametrize('name', ['local_costmap', 'global_costmap'])
def test_inflation_layer_is_enabled_in_plugins(params, name):
    assert 'inflation_layer' in costmaps(params)[name]['plugins']


def test_bt_parses_and_references_handle_box_and_escape():
    root = ET.parse(BT).getroot()
    assert root.tag == 'root'
    main = root.attrib['main_tree_to_execute']
    assert main in [bt.attrib['ID'] for bt in root.iter('BehaviorTree')]
    actions = {el.attrib.get('action_name') for el in root.iter() if 'action_name' in el.attrib}
    assert '/handle_box' in actions
    assert '/escape' in actions


def test_bt_has_no_removed_legacy_nodes_in_use():
    # visual_approach / sweep / push_through were folded into /handle_box: no live node calls them
    root = ET.parse(BT).getroot()
    actions = {el.attrib.get('action_name') for el in root.iter() if 'action_name' in el.attrib}
    assert not actions & {'/visual_approach', '/sweep', '/push_through'}


# ---------------------------------------------------------------- launch files
launch = pytest.importorskip('launch')
pytest.importorskip('launch_ros')
ament = pytest.importorskip('ament_index_python.packages')


def _load(filename, modname):
    import importlib.util
    spec = importlib.util.spec_from_file_location(modname, os.path.join(PKG_DIR, 'launch', filename))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


@pytest.fixture
def fake_prefix(tmp_path):
    lib = tmp_path / 'lib' / 'festa_demo'
    lib.mkdir(parents=True)
    for f in os.listdir(os.path.join(PKG_DIR, 'scripts')):
        if f.endswith('.py'):
            os.symlink(os.path.join(PKG_DIR, 'scripts', f), lib / f)
    return str(tmp_path)


def _real_share(pkg):
    return ament.get_package_share_directory(pkg)


def _patch_share(mod, monkeypatch, fake_prefix=None):
    def share(pkg):
        if pkg == 'festa_demo':
            return PKG_DIR
        return _real_share(pkg)
    monkeypatch.setattr(mod, 'get_package_share_directory', share)
    if fake_prefix:
        monkeypatch.setattr(mod, 'get_package_prefix', lambda pkg: fake_prefix)


def _text(x, ctx=None):
    """Plain text of a str / Substitution / list of them (LaunchConfiguration needs ctx)."""
    if isinstance(x, str):
        return x
    if hasattr(x, 'perform'):
        return x.perform(ctx) if ctx is not None else x.text
    return ''.join(_text(i, ctx) for i in x)


@pytest.fixture
def shuttle(monkeypatch):
    mod = _load('festa_shuttle.launch.py', 'festa_test_shuttle_launch')
    _patch_share(mod, monkeypatch)
    return mod


def _shuttle_parts(shuttle):
    ld = shuttle.generate_launch_description()
    decls = [e for e in ld.entities if isinstance(e, launch.actions.DeclareLaunchArgument)]
    incl = [e for e in ld.entities if isinstance(e, launch.actions.IncludeLaunchDescription)]
    assert len(incl) == 1
    return decls, incl[0]


def test_shuttle_round_trips_default_zero(shuttle):
    decls, _ = _shuttle_parts(shuttle)
    rt = [d for d in decls if d.name == 'round_trips']
    assert len(rt) == 1 and _text(rt[0].default_value, launch.LaunchContext()) == '0'


def test_shuttle_includes_festa_demo_launch(shuttle):
    _, incl = _shuttle_parts(shuttle)
    src = incl.launch_description_source
    ld = src.get_launch_description(launch.LaunchContext())     # imports the included file only
    assert src.location.endswith(os.path.join('launch', 'festa_demo.launch.py'))
    declared = {e.name for e in ld.entities if isinstance(e, launch.actions.DeclareLaunchArgument)}
    # every argument the shuttle passes must exist in the included launch file
    assert {'mode', 'pick', 'pick_first', 'shuttle', 'round_trips'} <= declared


def _shuttle_args(shuttle, ctx):
    decls, incl = _shuttle_parts(shuttle)
    for d in decls:
        d.visit(ctx)
    return {_text(k, ctx): _text(v, ctx) for k, v in incl.launch_arguments}


def test_shuttle_launch_arguments(shuttle):
    ctx = launch.LaunchContext()
    args = _shuttle_args(shuttle, ctx)
    assert args == {'mode': 'sim', 'pick': 'true', 'pick_first': 'true', 'shuttle': 'true',
                    'round_trips': '0'}


def test_shuttle_passes_user_overrides(shuttle):
    ctx = launch.LaunchContext()
    ctx.launch_configurations['mode'] = 'real'
    ctx.launch_configurations['round_trips'] = '3'
    decls, incl = _shuttle_parts(shuttle)
    # explicit command-line values win over the declared defaults (DeclareLaunchArgument keeps them)
    for d in decls:
        d.visit(ctx)
    args = {_text(k, ctx): _text(v, ctx) for k, v in incl.launch_arguments}
    assert args['mode'] == 'real' and args['round_trips'] == '3' and args['shuttle'] == 'true'


def _send_goal_cmd(demo, ctx):
    """Visit festa_demo.launch.py's declarations + OpaqueFunction; return the send_goal cmd as text."""
    ld = demo.generate_launch_description()
    produced = []
    for ent in ld.entities:
        if isinstance(ent, launch.actions.DeclareLaunchArgument):
            ent.visit(ctx)
        elif isinstance(ent, launch.actions.OpaqueFunction):
            produced = ent.visit(ctx)
    timers = [a for a in produced if isinstance(a, launch.actions.TimerAction)]
    assert len(timers) == 1
    proc = timers[0].actions[0]
    assert isinstance(proc, launch.actions.ExecuteProcess)
    return [_text(part, ctx) for part in proc.cmd]


def test_shuttle_into_festa_demo_send_goal_cmd(shuttle, monkeypatch, fake_prefix):
    demo = _load('festa_demo.launch.py', 'festa_test_demo_launch')
    _patch_share(demo, monkeypatch, fake_prefix)
    ctx = launch.LaunchContext()
    # 1. the shuttle launch's declared defaults and its include arguments ...
    args = _shuttle_args(shuttle, ctx)
    # 2. ... become the launch configurations of festa_demo.launch.py (what an include does)
    ctx2 = launch.LaunchContext()
    for k, v in args.items():
        ctx2.launch_configurations[k] = v
    cmd = _send_goal_cmd(demo, ctx2)
    assert cmd[1].endswith('send_goal.py')
    for expect in ('shuttle:=true', 'round_trips:=0', 'pick_first:=true', 'return_to_start:=true',
                   'send_goal:=true', 'set_initial_pose:=true', 'use_sim_time:=true'):
        assert expect in cmd, expect


def test_festa_demo_defaults_are_not_shuttle(monkeypatch, fake_prefix):
    demo = _load('festa_demo.launch.py', 'festa_test_demo_launch2')
    _patch_share(demo, monkeypatch, fake_prefix)
    cmd = _send_goal_cmd(demo, launch.LaunchContext())
    assert 'shuttle:=false' in cmd and 'round_trips:=1' in cmd and 'pick_first:=true' in cmd


def test_festa_demo_pick_first_false_when_pick_false(monkeypatch, fake_prefix):
    demo = _load('festa_demo.launch.py', 'festa_test_demo_launch3')
    _patch_share(demo, monkeypatch, fake_prefix)
    ctx = launch.LaunchContext()
    ctx.launch_configurations['pick'] = 'false'
    cmd = _send_goal_cmd(demo, ctx)
    assert 'pick_first:=false' in cmd


def test_festa_demo_rejects_unknown_mode(monkeypatch, fake_prefix):
    demo = _load('festa_demo.launch.py', 'festa_test_demo_launch4')
    _patch_share(demo, monkeypatch, fake_prefix)
    ctx = launch.LaunchContext()
    ctx.launch_configurations['mode'] = 'bogus'
    with pytest.raises(RuntimeError):
        _send_goal_cmd(demo, ctx)
