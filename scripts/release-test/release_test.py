"""AeroCore 0.41 / hdfv 0.24: the release JARs in a clean NeoForge, as players run them (FMLEnvironment.production):
a dedicated server and a client installed by NeoForge's own installer, outside the projects and the user's launcher
(%USERPROFILE%/.mcmod/release-test). Downloads: the installer (maven.neoforged.net), and what it and the client need
from Mojang and NeoForge; the assets come from the Gradle cache the development runs filled. JEI and Create come from
the Gradle cache too.

    py -3 scripts/release-test/release_test.py setup
    py -3 scripts/release-test/release_test.py server <base|jei|create>      starts, runs a few commands, stops, checks the log
    py -3 scripts/release-test/release_test.py client <base|jei> [plan]      an AutoShot plan file (the world must be in saves)
    py -3 scripts/release-test/release_test.py multiplayer [plan]            the server (jei) with the client joining it

The client loads AeroCore's development tools mod too (AutoShot, the window on monitor 2, no focus): it is never
released, but it drives the client here. Everything else in mods is what players get. The server's EULA is the
user's (accepted for the local test servers, hdfv 0.14); online-mode is off for the local client.
"""
import glob
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import threading
import time
import urllib.request
import uuid

HOME = os.path.expanduser('~')
ROOT = os.path.join(HOME, '.mcmod', 'release-test')
JAVA = os.path.join(HOME, '.mcmod', 'java', 'jdk-21.0.12.1+1', 'bin', 'java.exe')
TMP = os.path.join(HOME, '.mcmod', 'tmp')
NEO = '21.1.252'
INSTALLER_URL = 'https://maven.neoforged.net/releases/net/neoforged/neoforge/%s/neoforge-%s-installer.jar' % (NEO, NEO)
ASSETS = os.path.join(HOME, '.gradle', 'caches', 'neoformruntime', 'assets')
GRADLE = os.path.join(HOME, '.gradle', 'caches', 'modules-2', 'files-2.1')
AEROCORE = os.path.join(HOME, '.mcmod', 'build', 'aerocore-e6babf45', 'neoforge', 'libs')
HDFV = os.path.join(HOME, '.mcmod', 'build', 'hdfv-99dac8aa', 'neoforge', 'libs')
HDFV_CLIENT_SAVES = os.path.join(HOME, '.mcmod', 'runs', 'hdfv', 'client', 'saves')
ERRORS = re.compile(r'NoClassDefFoundError|ClassNotFoundException|Mixin.*(?:error|failed)|Exception in|Missing or unsupported mandatory|'
                    r'Attempted to load class net/minecraft/client|has failed to load correctly|Crash report|\[[^\]]*/ERROR\]', re.I)


def log(*a):
    print(*a, flush=True)


def download(url, path, sha1=None):
    if os.path.exists(path) and (sha1 is None or hashlib.sha1(open(path, 'rb').read()).hexdigest() == sha1):
        return
    os.makedirs(os.path.dirname(path), exist_ok=True)
    log('download', url)
    with urllib.request.urlopen(url, timeout=120) as r:
        data = r.read()
    if sha1 is not None and hashlib.sha1(data).hexdigest() != sha1:
        sys.exit('checksum mismatch: ' + url)
    with open(path, 'wb') as f:
        f.write(data)


def jar(folder, prefix, version_from=None):
    files = [f for f in glob.glob(os.path.join(folder, prefix + '*.jar')) if not f.endswith('-sources.jar')]
    if not files:
        sys.exit('no ' + prefix + ' in ' + folder)
    return max(files, key=os.path.getmtime)


def cached(group, name, contains=''):
    files = [f for f in glob.glob(os.path.join(GRADLE, group, name, '*', '*', '*.jar')) if contains in os.path.basename(f) and 'sources' not in f and 'api' not in os.path.basename(f)]
    if not files:
        sys.exit('not in the Gradle cache: %s:%s' % (group, name))
    return max(files, key=os.path.getmtime)


def mods(case, client):
    out = [jar(AEROCORE, 'aerocore-neoforge-1.21.1-'), jar(HDFV, 'hdfv-neoforge-1.21.1-')]
    if client:
        out.append(jar(AEROCORE, 'aerocore-devtools-1.21.1-'))
    if case in ('jei', 'create'):
        out.append(cached('mezz.jei', 'jei-1.21.1-neoforge'))
    if case == 'create':
        out.append(cached('com.simibubi.create', 'create-1.21.1'))
    return out


def put_mods(folder, jars):
    os.makedirs(folder, exist_ok=True)
    for f in os.listdir(folder):
        os.remove(os.path.join(folder, f))
    for j in jars:
        shutil.copy2(j, folder)
    log('mods:', ', '.join(os.path.basename(j) for j in jars))


# ------------------------------------------------------------------------------------------------ setup

def setup():
    installer = os.path.join(ROOT, 'installer', 'neoforge-%s-installer.jar' % NEO)
    download(INSTALLER_URL, installer)
    server = os.path.join(ROOT, 'server')
    if not os.path.exists(os.path.join(server, 'libraries', 'net', 'neoforged', 'neoforge', NEO, 'win_args.txt')):
        os.makedirs(server, exist_ok=True)
        subprocess.run([JAVA, '-jar', installer, '--installServer', server], check=True, cwd=server)
    client = os.path.join(ROOT, 'client')
    if not os.path.exists(os.path.join(client, 'versions', 'neoforge-' + NEO, 'neoforge-%s.json' % NEO)):
        os.makedirs(client, exist_ok=True)
        profiles = os.path.join(client, 'launcher_profiles.json')
        if not os.path.exists(profiles):
            with open(profiles, 'w') as f:
                json.dump({'profiles': {}}, f)
        subprocess.run([JAVA, '-jar', installer, '--installClient', client], check=True, cwd=client)
    # the client's own libraries (the launcher fetches them; the installer only its own)
    for lib in libraries(client):
        if lib['url']:
            download(lib['url'], lib['file'], lib['sha1'])
    log('setup done:', ROOT)


def allowed(rules):
    if not rules:
        return True
    ok = False
    for r in rules:
        osr = r.get('os', {})
        if osr and osr.get('name') not in (None, 'windows'):
            continue
        if 'features' in r:
            continue
        ok = r['action'] == 'allow'
    return ok


def version(client, name):
    return json.load(open(os.path.join(client, 'versions', name, name + '.json'), encoding='utf-8'))


def libraries(client):
    neo = version(client, 'neoforge-' + NEO)
    vanilla = version(client, neo['inheritsFrom'])
    out, seen = [], set()
    for v in (neo, vanilla):
        for lib in v.get('libraries', []):
            if not allowed(lib.get('rules')):
                continue
            art = lib.get('downloads', {}).get('artifact')
            if not art:
                continue
            key = lib['name']
            if key in seen:
                continue
            seen.add(key)
            out.append({'name': key, 'file': os.path.join(client, 'libraries', art['path'].replace('/', os.sep)), 'url': art.get('url'), 'sha1': art.get('sha1')})
    return out


# ------------------------------------------------------------------------------------------------ the server

def server_run(case, keep_running=None):
    folder = os.path.join(ROOT, 'server')
    put_mods(os.path.join(folder, 'mods'), mods(case, False))
    with open(os.path.join(folder, 'eula.txt'), 'w') as f:
        f.write('# accepted by the user for the local test servers (hdfv 0.14)\neula=true\n')
    props = {'online-mode': 'false', 'level-name': 'AeroCore Release Server QA', 'motd': 'hdfv release check', 'spawn-protection': '0',
             'server-port': '25565', 'view-distance': '8'}
    with open(os.path.join(folder, 'server.properties'), 'w') as f:
        f.write(''.join('%s=%s\n' % kv for kv in props.items()))
    with open(os.path.join(folder, 'user_jvm_args.txt'), 'w') as f:
        # an argument file splits at spaces: the user's folder may have some
        f.write('-Xmx4G\n"-Djava.io.tmpdir=%s"\n' % TMP.replace('\\', '/'))
    args = [JAVA, '@user_jvm_args.txt', '@libraries/net/neoforged/neoforge/%s/win_args.txt' % NEO, 'nogui']
    started = time.time()
    p = subprocess.Popen(args, cwd=folder, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8', errors='replace')
    ready = threading.Event()
    lines = []

    def read():
        for line in p.stdout:
            lines.append(line)
            if 'Done (' in line and 'For help' in line:
                ready.set()
    threading.Thread(target=read, daemon=True).start()
    while not ready.wait(5):
        if p.poll() is not None or time.time() - started > 600:
            if p.poll() is None:
                p.kill()
            log(''.join(lines[-40:]))
            sys.exit('the server did not start')
    log('server up')
    for c in ['list', 'aerocore wind', 'hdfv vehicle samples', 'hdfv decal list', 'save-all']:
        p.stdin.write(c + '\n')
        p.stdin.flush()
        time.sleep(2)
    if keep_running is not None:
        keep_running()
    p.stdin.write('stop\n')
    p.stdin.flush()
    p.wait(300)
    check(os.path.join(folder, 'logs', 'latest.log'), 'server ' + case)


def check(path, what):
    text = open(path, encoding='utf-8', errors='replace').read()
    bad = [l for l in text.splitlines() if ERRORS.search(l)]
    log('%s: %d lines in %s, %d suspicious' % (what, text.count('\n'), path, len(bad)))
    for l in bad[:40]:
        log('   ', l[:300])
    return bad


# ------------------------------------------------------------------------------------------------ the client

def client_run(case, plan, server=False):
    base = os.path.join(ROOT, 'client')
    game = os.path.join(base, 'game-' + case)
    put_mods(os.path.join(game, 'mods'), mods(case, True))
    cfg = os.path.join(game, 'config')
    os.makedirs(cfg, exist_ok=True)
    with open(os.path.join(cfg, 'fml.toml'), 'w') as f:
        f.write('earlyWindowControl = false\n')
    options = os.path.join(game, 'options.txt')
    if not os.path.exists(options):
        with open(options, 'w') as f:
            f.write('lang:en_us\npauseOnLostFocus:false\nonboardAccessibility:false\ntutorialStep:none\nskipMultiplayerWarning:true\njoinedFirstServer:true\n')
    world = os.path.join(game, 'saves', 'AeroCore Production QA')
    if not os.path.exists(world):
        shutil.copytree(os.path.join(HDFV_CLIENT_SAVES, 'AeroCore HDFV QA'), world)
        lock = os.path.join(world, 'session.lock')
        if os.path.exists(lock):
            os.remove(lock)
    if plan:
        shutil.copy(plan, os.path.join(game, 'aerocore-autoshot.txt'))
    neo = version(base, 'neoforge-' + NEO)
    vanilla = version(base, neo['inheritsFrom'])
    sep = ';'
    # not the vanilla client JAR: NeoForge loads its patched Minecraft from the libraries (both would export net.minecraft)
    cp = [l['file'] for l in libraries(base)]
    name = 'QA'
    values = {
        'natives_directory': os.path.join(base, 'natives'), 'launcher_name': 'hdfv-release-test', 'launcher_version': '1',
        'classpath': sep.join(cp), 'classpath_separator': sep, 'library_directory': os.path.join(base, 'libraries'),
        'version_name': neo['id'], 'auth_player_name': name, 'game_directory': game, 'assets_root': ASSETS,
        'assets_index_name': vanilla['assetIndex']['id'], 'auth_uuid': str(uuid.uuid3(uuid.NAMESPACE_OID, 'OfflinePlayer:' + name)).replace('-', ''),
        'auth_access_token': '0', 'clientid': '0', 'auth_xuid': '0', 'user_type': 'msa', 'version_type': 'release',
    }

    def expand(args):
        out = []
        for a in args:
            if isinstance(a, dict):
                if not allowed(a.get('rules')):
                    continue
                vals = a['value'] if isinstance(a['value'], list) else [a['value']]
            else:
                vals = [a]
            for v in vals:
                out.append(re.sub(r'\$\{([a-z_]+)\}', lambda m: values.get(m.group(1), m.group(0)), v))
        return out
    jvm = expand(vanilla['arguments']['jvm']) + expand(neo['arguments'].get('jvm', []))
    jvm += ['-Xmx4G', '-Daerocore.window.monitor=2', '-Djava.io.tmpdir=' + TMP]
    game_args = expand(vanilla['arguments']['game']) + expand(neo['arguments'].get('game', []))
    cmd = [JAVA] + jvm + [neo['mainClass']] + game_args
    log('client', case, 'in', game)
    started = time.time()
    p = subprocess.run(cmd, cwd=game, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    log('client exited %d after %.0f s' % (p.returncode, time.time() - started))
    check(os.path.join(game, 'logs', 'latest.log'), 'client ' + case)


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    what = sys.argv[1]
    if what == 'setup':
        setup()
    elif what == 'server':
        server_run(sys.argv[2] if len(sys.argv) > 2 else 'base')
    elif what == 'client':
        client_run(sys.argv[2] if len(sys.argv) > 2 else 'base', sys.argv[3] if len(sys.argv) > 3 else None)
    elif what == 'multiplayer':
        plan = sys.argv[2] if len(sys.argv) > 2 else None
        server_run('jei', keep_running=lambda: client_run('jei', plan, server=True))
    else:
        sys.exit(__doc__)


if __name__ == '__main__':
    main()
