#!/usr/bin/env python3
# coding=utf-8

'''
# @copyright Copyright (c) 2022, HiSilicon (Shanghai) Technologies Co., Ltd. All rights reserved.
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
# following conditions are met:
# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
# disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
# following disclaimer in the documentation and/or other materials provided with the distribution.
# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
# products derived from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
# INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
# WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
# USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import os
import stat
import sys
import json
import configparser
import subprocess
import tarfile
import zipfile
import shutil
import pathlib


def set_bool_parameter(parameter, value, compile_parameter):
    '''
    Function description: Set bool parameter, yes is true, no is false
    '''

    if parameter == "yes":
        if value not in compile_parameter:
            compile_parameter.append(value)
    else:
        if value in compile_parameter:
            compile_parameter.remove(value)


def un_alltools(file_name):
    '''
    Function description: Decompress all compilation tools.
    '''
    if len(str(file_name)) > 256:
        print("Error!!!! The file path string is too long(256): ", file_name)
        return

    for word in file_name:
        if '.' in word:
            suffix = file_name.split('.')[-1]
            if suffix in ('gz', 'tar'):
                untar(file_name)
            elif suffix == 'zip':
                unzip(file_name)
            break


def untar(file_name):
    '''
    Function description: Decompress the tar or tar.gz file.
    '''

    head, tail = os.path.split(file_name)
    cur_path = os.path.join(head, tail.split('.')[0])

    if not os.path.exists(cur_path):
        with tarfile.open(file_name) as tar:
            tar.extractall(os.path.dirname(cur_path))


def unzip(file_name):
    '''
    Function description: Decompress the zip file.
    '''

    max_size = 1 * 1024 * 1024 * 500
    cur_size = 0

    with zipfile.ZipFile(file_name) as zip_file:
        head, tail = os.path.split(file_name)
        file_name = os.path.join(head, tail.split('.')[0])

        if not os.path.isdir(file_name):
            os.mkdir(file_name)

        for names in zip_file.infolist():
            cur_size += names.file_size
            if cur_size > max_size:
                break
            zip_file.extract(names.filename, file_name)


def config_toolchain(tool_path, sdk_path):
    files1 = (os.listdir(tool_path))
    files2 = (os.listdir(sdk_path))
    files1.sort()
    files2.sort()

    for item in files2:
        if item not in files1:
            source = os.path.join(sdk_path, item)
            if os.path.isdir(source):
                continue
            target = os.path.join(tool_path, item)
            shutil.copyfile(source, target)
            un_alltools(target)

    gn_path = os.path.join(tool_path, "gn-win", "gn.exe")
    gnzip_path = os.path.join(tool_path, "gn-win.zip")
    if not os.path.exists(gn_path):
        un_alltools(gnzip_path)

    ninja_path = os.path.join(tool_path, "ninja-win", "ninja.exe")
    ninjazip_path = os.path.join(tool_path, "ninja-win.zip")
    if not os.path.exists(ninja_path):
        un_alltools(ninjazip_path)


def config_download_toolchain(tool_path):
    del_list = [f for f in os.listdir(tool_path)]
    for f in del_list:
        file_path = os.path.join(tool_path, f)
        if os.path.isdir(file_path):
            shutil.rmtree(file_path)

    cc_riscv32_win_env_tar_gz_path = os.path.join(
        tool_path, "cc_riscv32_win_env.tar.gz")
    un_alltools(cc_riscv32_win_env_tar_gz_path)

    cc_riscv32_musl_win_tar_gz_path = os.path.join(
        tool_path, "cc_riscv32_musl_win.tar.gz")
    un_alltools(cc_riscv32_musl_win_tar_gz_path)

    cc_riscv32_musl_fp_win_tar_gz_path = os.path.join(
        tool_path, "cc_riscv32_musl_fp_win.tar.gz")
    un_alltools(cc_riscv32_musl_fp_win_tar_gz_path)

    gnzip_path = os.path.join(tool_path, "gn-win.zip")
    un_alltools(gnzip_path)

    ninjazip_path = os.path.join(tool_path, "ninja-win.zip")
    un_alltools(ninjazip_path)

    path1 = os.path.join(tool_path, "bundle.json")
    path2 = os.path.join(tool_path, "LICENSE")
    path3 = os.path.join(tool_path, "README.md")
    if os.path.exists(path1):
        os.remove(path1)
    if os.path.exists(path2):
        os.remove(path2)
    if os.path.exists(path3):
        os.remove(path3)


def config_win_env(tool_path):
    source = os.path.join(tool_path, "cc_riscv32_win_env")
    if not os.path.exists(source):
        return

    target1 = os.path.join(tool_path, "cc_riscv32_musl_fp_win\\bin")
    target2 = os.path.join(tool_path, "cc_riscv32_musl_win\\bin")
    src_files = os.listdir(source)
    for file_name in src_files:
        full_file_name = os.path.join(source, file_name)
        if not os.path.isfile(full_file_name):
            continue
        if os.path.exists(target1):
            shutil.copy(full_file_name, target1)
        if os.path.exists(target2):
            shutil.copy(full_file_name, target2)


def config_gdb_env(tool_path):
    tool_chain = ["cc_riscv32_musl_fp_win", "cc_riscv32_musl_win"]
    rename_bin = [
        ["riscv32-linux-musl-gdb.exe", "riscv32-unknown-elf-gdb.exe"],
        ["riscv32-linux-musl-objdump.exe", "riscv32-unknown-elf-objdump.exe"]
    ]

    for tool in tool_chain:
        for bins in rename_bin:
            old = os.path.join(tool_path, tool, "bin", bins[0])
            new = os.path.join(tool_path, tool, "bin", bins[1])

            if os.path.exists(old) and not os.path.exists(new):
                shutil.copyfile(old, new)


def convert_file_to_path(includes):
    path = [i for i in includes.replace('\n', '').split(",") if i != '']

    for i, item in enumerate(path):
        if os.path.isfile(item):
            path[i] = os.path.dirname(item)
    return list(set(path))


def convert_file_to_path_extern(includes, cur_path):
    # Processing Path
    path = [i for i in includes.replace('\n', '').split(",") if i != '']
    # Process only paths outside the project
    path = [i for i in path if cur_path not in i]

    if not path:
        return path

    for i, item in enumerate(path):
        if os.path.isfile(item):
            path[i] = os.path.dirname(item)

    return list(set(path))


class CompileParameter:
    optimization = ''
    warning = ''
    werror = ''
    wno_unused_function = ''
    wno_unused_label = ''
    wno_unused_parameter = ''
    wno_unused_variable = ''
    fstack = ''

    def __init__(self):
        pass

    def set_optimization(self, config):
        self.optimization = config

    def set_warnning(self, config):
        self.warnning = config

    def set_werror(self, config):
        self.werror = config

    def set_function(self, config):
        self.wno_unused_function = config

    def set_label(self, config):
        self.wno_unused_label = config

    def set_parameter(self, config):
        self.wno_unused_parameter = config

    def set_variable(self, config):
        self.wno_unused_variable = config

    def set_fstack(self, config):
        self.fstack = config

    @staticmethod
    def set_generate_crc(value):
        deveco_path = 'build/config.ini'
        if not os.path.exists(deveco_path):
            return

        config = configparser.ConfigParser()
        config.read(deveco_path)

        if not config.has_section('gn_args'):
            return

        config.set('gn_args', 'gen_crc', value)
        modes = stat.S_IWUSR | stat.S_IRUSR
        flags = os.O_RDWR | os.O_CREAT
        config.write(os.fdopen(os.open(deveco_path, flags, modes), 'w'))


    @staticmethod
    def set_build_type(build_type):
        command = 'python build/build.py -b ' + build_type
        process = subprocess.Popen(command, shell=False)
        process.wait()

    @staticmethod
    def check_toolchain(sdk_path):
        ide_path = ".deveco-device-tool/compiler_tool_chain/Windows"
        tool_path = os.path.join(os.path.expanduser("~"), ide_path)
        sdk_path = os.path.join(sdk_path, "tools/toolchain/Windows")
        ninja_exe_path = os.path.join(tool_path, "ninja-win", "ninja.exe")
        # check toolchain
        all_times = 3
        times = 0
        if not os.path.exists(sdk_path):
            for times in range(0, all_times):
                if not os.path.exists(ninja_exe_path):
                    config_download_toolchain(tool_path)
                    config_win_env(tool_path)
                    config_gdb_env(tool_path)
                    times += 1
                else:
                    return
            if not os.path.exists(ninja_exe_path):
                print("Packages in " + tool_path +
                      " are damaged! Please manually delete and rebuild!")
            return

        if not os.path.exists(os.path.dirname(tool_path)):
            os.mkdir(os.path.dirname(tool_path))
        if not os.path.exists(tool_path):
            os.mkdir(tool_path)
        else:
            if check_tool_chain_md5(sdk_path, tool_path):
                return

        config_toolchain(tool_path, sdk_path)
        config_win_env(tool_path)
        config_gdb_env(tool_path)

    @staticmethod
    def set_toolchain(toolchian, build_type):
        if toolchian == 'cc_riscv32_musl_fp_win':
            command = 'python build/build.py -t hcc_fpu -b' + build_type
        else:
            command = 'python build/build.py -t hcc -b' + build_type

        process = subprocess.Popen(command, shell=False)
        process.wait()

    def set_compile_parameter(self):
        '''
        Function description: Set compile parameter.
        '''

        file_data = ""
        file_path = "chip/target/userconfig.json"

        modes = stat.S_IWUSR | stat.S_IRUSR
        flags = os.O_RDWR | os.O_CREAT
        with os.fdopen(os.open(file_path, flags, modes), 'r+') as f:
            file_data = f.read()
            content = json.loads(file_data)

            for subsystem in content["system"][0]["subsystem"]:
                if subsystem["name"] != "compile_frame":
                    continue

                param = subsystem["cflags"]
                param[0] = f'-{self.optimization}'

                if self.warnning == "yes":
                    param[2] = "-Wall"
                else:
                    param[2] = "-w"

                set_bool_parameter(
                    self.werror, "-Werror", param)
                set_bool_parameter(
                    self.wno_unused_function, "-Wno-unused-function", param)
                set_bool_parameter(self.wno_unused_label,
                                   "-Wno-unused-label", param)
                set_bool_parameter(
                    self.wno_unused_parameter, "-Wno-unused-parameter", param)
                set_bool_parameter(
                    self.wno_unused_variable, "-Wno-unused-variable", param)
                set_bool_parameter(
                    self.fstack, "-fstack-protector-strong", param)

                file_data = json.dumps(
                    content, indent=4, ensure_ascii=False)

                f.seek(0)
                f.truncate()
                f.write(file_data)
                break

    @staticmethod
    def set_static_library(enable, name, sources, includes):
        '''
        Function description: Set static library.
        '''
        file_data = ""
        file_path = "chip/target/userconfig.json"
        cflags = []
        asmflags = []
        ldflags = []

        modes = stat.S_IWUSR | stat.S_IRUSR
        flags = os.O_RDWR | os.O_CREAT
        with os.fdopen(os.open(file_path, flags, modes), 'r+') as f:
            file_data = f.read()
            content = json.loads(file_data)

            for subsystem in content["system"][0]["subsystem"]:
                if subsystem["name"] != "compile_frame":
                    continue
                if enable == "no":
                    name = ""
                    sources = ""
                    includes = ""
                    continue

                cflags = subsystem["cflags"]
                asmflags = subsystem["asmflags"]
                ldflags = subsystem["ldflags"]
                break

            for subsystem in content["system"][0]["subsystem"]:
                if subsystem["name"] != "static_lib":
                    continue

                for component in subsystem["component"]:
                    component["cflags"] = cflags
                    component["asmflags"] = asmflags
                    component["ldflags"] = ldflags
                    component["name"] = name
                    component["sources"] = [i for i in sources.replace(
                        '\n', '').split(",") if i != '']
                    component["includes"] = convert_file_to_path(includes)

            file_data = json.dumps(content, indent=4, ensure_ascii=False)

            f.seek(0)
            f.truncate()
            f.write(file_data)

    @staticmethod
    def set_extern_staticlib_link(libspath, libsinclude):
        '''
        Function description: Set static library.
        '''
        file_data = ""
        file_path = "chip/target/userconfig.json"
        ext_path = []
        ext_include = []
        cur_path = str(pathlib.Path.cwd())

        modes = stat.S_IWUSR | stat.S_IRUSR
        flags = os.O_RDWR | os.O_CREAT
        with os.fdopen(os.open(file_path, flags, modes), 'r+') as f:
            file_data = f.read()
            content = json.loads(file_data)

            for subsystem in content["system"][0]["subsystem"]:
                if subsystem["name"] != "compile_frame":
                    continue

                subsystem['extlibspath'] = []
                subsystem['extlibsname'] = []
                subsystem['extlibsinclude'] = []

                if not libspath:
                    break
                # Processing Path
                ext_path = [i for i in libspath.replace(
                    '\n', '').split(",") if i != '']
                # Process only paths outside the project
                ext_path = [i for i in ext_path if cur_path not in i]
                if ext_path:
                    subsystem['extlibspath'] = [os.path.dirname(i)
                        for i in ext_path]
                    subsystem['extlibsname'] = [os.path.basename(i)
                        for i in ext_path]
                    # List deduplication
                    subsystem['extlibspath'] = list(
                        set(subsystem['extlibspath']))
                    subsystem['extlibsname'] = list(
                        set(subsystem['extlibsname']))
                if not libsinclude:
                    break
                subsystem['extlibsinclude'] = convert_file_to_path_extern(
                    libsinclude, cur_path)

            file_data = json.dumps(content, indent=4, ensure_ascii=False)

            f.seek(0)
            f.truncate()
            f.write(file_data)


def check_tool_chain_md5(sdk_path, tool_path):
    tool_name = os.path.join(tool_path, "ninja-win", "ninja.exe")
    if not os.path.exists(tool_name):
        print("Tool chain is damaged! Repairing in progress.")
        del_file(tool_path)
        return False

    file_name = os.path.join(sdk_path, "cc_riscv32_musl_fp_win")
    if not os.path.exists(file_name):
        release_tool_chain(sdk_path)

    sdk_md5 = get_toolchain_md5(os.path.join(file_name, "bin"))

    tool_name = os.path.join(tool_path, "cc_riscv32_musl_fp_win")
    tool_zip_name = os.path.join(tool_path, "cc_riscv32_musl_fp_win.tar.gz")
    if os.path.exists(tool_zip_name) and os.path.exists(tool_name):
        tool_md5 = get_toolchain_md5(os.path.join(tool_name, "bin"))
        if sdk_md5 == tool_md5 or\
                sdk_md5 == "noMD5" or\
                tool_md5 == "noMD5":
            return True

    del_file(tool_path)
    return False


def release_tool_chain(sdk_path):
    for item in os.listdir(sdk_path):
        if "cc_riscv32_musl_fp_win" in item or "cc_riscv32_win_env" in item:
            zip_name = os.path.join(sdk_path, item)
            if os.path.isdir(zip_name):
                continue
            un_alltools(zip_name)

    target = os.path.join(sdk_path, "cc_riscv32_musl_fp_win", "bin")
    source = os.path.join(sdk_path, "cc_riscv32_win_env")
    if not os.path.exists(source):
        return

    for file_name in os.listdir(source):
        full_file_name = os.path.join(source, file_name)
        if not os.path.isfile(full_file_name):
            continue
        if os.path.exists(target):
            shutil.copy(full_file_name, target)


def del_file(filepath):
    '''
    Delete all files or folders in a directory.
    '''
    del_list = [f for f in os.listdir(filepath) if "cc_riscv32" in f]
    for f in del_list:
        file_path = os.path.join(filepath, f)
        if os.path.isfile(file_path):
            os.remove(file_path)
        elif os.path.isdir(file_path):
            shutil.rmtree(file_path)


def get_toolchain_md5(target_path):
    cur_path = pathlib.Path.cwd()

    if not os.path.exists(target_path):
        return "noMD5"

    os.chdir(target_path)
    # Check the MD5 value.
    cmd = ['certutil', '-hashfile', 'riscv32-linux-musl-gcc.exe', 'md5']
    out_bytes = subprocess.check_output(cmd, shell=False)
    os.chdir(cur_path)
    return out_bytes


def extern_static_library(env, config, param):
    '''
    Function description: Linking External Static Libraries.
    '''

    if 'extern_staticlib_path' in config[env] and \
        'extern_staticlib_include' in config[env]:
        param.set_extern_staticlib_link(
            config[env]['extern_staticlib_path'],
            config[env]['extern_staticlib_include'])


def fix_warning(config, env, deveco_path):
    if 'warnning' in config[env]:
        config.set(env, 'warning', config[env]['warnning'])
        config.remove_option(env, 'warnning')
        modes = stat.S_IWUSR | stat.S_IRUSR
        flags = os.O_RDWR | os.O_CREAT
        with os.fdopen(os.open(deveco_path, flags, modes), 'w') as file:
            file.seek(0)
            file.truncate()
            config.write(file)


def build_check(argv):
    '''
    Function description: check compile parameter and tool chain.
    '''

    if len(argv) < 2:
        return

    deveco_path = '.deveco/deveco.ini'
    chip = argv[1]
    env = "env:" + chip
    config = configparser.ConfigParser()
    config.read(deveco_path)

    param = CompileParameter()
    if 'chip_package_path' in config[env]:
        param.check_toolchain(config[env]['chip_package_path'])

    if 'compiler' in config[env] and 'build_type' in config[env]:
        param.set_toolchain(config[env]['compiler'], config[env]['build_type'])

    if 'optimization' in config[env] and \
        'werror' in config[env] and \
        'wno_unused_function' in config[env] and \
        'wno_unused_label' in config[env] and \
        'wno_unused_parameter' in config[env] and \
        'wno_unused_variable' in config[env] :

        param.set_optimization(config[env]['optimization'])
        param.set_werror(config[env]['werror'])
        param.set_function(config[env]['wno_unused_function'])
        param.set_label(config[env]['wno_unused_label'])
        param.set_parameter(config[env]['wno_unused_parameter'])
        param.set_variable(config[env]['wno_unused_variable'])

    if 'fstack_protector_strong' in config[env]:
        param.set_fstack(config[env]['fstack_protector_strong'])

    fix_warning(config, env, deveco_path)

    if 'warning' in config[env]:
        param.set_warnning(config[env]['warning'])
    param.set_compile_parameter()

    if 'static_library_enable' in config[env] and \
        'static_library_name' in config[env] and \
        'static_library_source_file' in config[env] and \
            'static_library_dependency_header_file' in config[env]:
        param.set_static_library(
            config[env]['static_library_enable'],
            config[env]['static_library_name'],
            config[env]['static_library_source_file'],
            config[env]['static_library_dependency_header_file'])

    extern_static_library(env, config, param)

    if 'generate_crc' in config[env]:
        param.set_generate_crc(config[env]['generate_crc'])

    elf_path = 'out/bin/elf.json'
    if os.path.exists(elf_path):
        os.remove(elf_path)

if __name__ == "__main__":
    sys.exit(build_check(sys.argv))
