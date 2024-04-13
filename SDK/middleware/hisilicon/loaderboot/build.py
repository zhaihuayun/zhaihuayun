# !/usr/bin/env python
# -*- coding: utf-8 -*-

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
# build.py Function implementation: Build the compilation framework and
# compile the project.

import sys
import os
import pathlib
import subprocess
import shlex
import shutil
import stat
import re


def copy_code(cpy_dir):
    '''
    Function description: Code copy phase for CI build work.
    '''
    src_list = []
    target_list = []
    cur_path = pathlib.Path.cwd()
    base = os.path.realpath(cur_path.joinpath("../../.."))
    for i in cpy_dir:
        src_list.append(os.path.realpath(cur_path.joinpath(base, i)))
        target_list.append(os.path.realpath(cur_path.joinpath(cur_path, i)))

    index = 0
    for dir_path in src_list:
        if pathlib.Path(dir_path).exists():
            if not pathlib.Path(target_list[index]).exists():
                shutil.copytree(dir_path, target_list[index])
        index += 1


def get_chipname_in_gn(gn):
    with open(gn, 'r', encoding='UTF-8') as f:
        buf = f.readlines()
        for line in buf:
            if re.search(r'//chip', line) != None:
                chip_name = line.split('/')[3]
                return chip_name
    raise Exception('Error: Get Chip Name fail.')


def replace_string_in_files(gn, chipname):
    file_data = ''
    with open(gn, "r", encoding='UTF-8') as f:
        buf = f.readlines()
        for line in buf:
            line = line.replace(chipname['oldname'], chipname['newname'])
            file_data += line
    flags = os.O_WRONLY | os.O_CREAT | os.O_TRUNC
    modes = stat.S_IWUSR | stat.S_IRUSR
    with os.fdopen(os.open(gn, flags, modes), 'w+') as gn_file:
        gn_file.writelines(file_data)


def modify_build_gn():
    cur_path = pathlib.Path.cwd()
    chip_dir = os.path.realpath(cur_path.joinpath('chip'))
    dirs = os.listdir(chip_dir)
    build_gn = os.path.realpath(cur_path.joinpath('BUILD.gn'))
    old_chip_name = get_chipname_in_gn(build_gn)
    for chip_name in dirs:
        if chip_name != 'target':
            replace_dict = {'newname':chip_name, 'oldname':old_chip_name}
            replace_string_in_files(build_gn, replace_dict)
            return
    raise Exception('Error: Get Chip Dir Fail.')


def main(argv):
    '''
    Function description: build and compile entry function.
    '''
    source_dir = ['chip',
                  'drivers',
                  'generatecode']
    copy_code(source_dir)
    modify_build_gn()

    cmd = shlex.split('gn gen out --root=. --dotfile=build/\.gn')
    proc = subprocess.Popen(cmd, shell=False)
    proc.wait()
    ret_code = proc.returncode
    if ret_code != 0:
        raise Exception("CI {} failed, return code is {}".format(cmd, ret_code))

    cmd = shlex.split('ninja -C out')
    proc = subprocess.Popen(cmd, shell=False)
    proc.wait()
    ret_code = proc.returncode
    if ret_code != 0:
        raise Exception("CI {} failed, return code is {}".format(cmd,
                                                                 ret_code))

    file_abspath = pathlib.Path().cwd().joinpath("out", "bin", "loader.elf")
    bin_abspath = pathlib.Path().cwd().joinpath("loader.bin")
    cmd = ['riscv32-linux-musl-objcopy', '-Obinary',
           str(file_abspath), str(bin_abspath)]
    process = subprocess.Popen(cmd, shell=False)
    process.wait()
    ret_code = process.returncode
    if ret_code != 0:
        raise Exception("{} failed, return code is {}".format('bin_file',
                        ret_code))

    list_path = file_abspath.parent.joinpath('{}.list'
                                             .format(file_abspath.stem))
    if list_path.exists():
        os.remove(list_path)
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    modes = stat.S_IWUSR | stat.S_IRUSR
    with os.fdopen(os.open(list_path, flags, modes), 'w+') as list_file:
        cmd = ['riscv32-linux-musl-objdump', '-S', str(file_abspath)]
        process = subprocess.Popen(cmd, stdout=list_file, shell=False)
        process.wait()
        ret_code = process.returncode
        if ret_code != 0:
            raise Exception("{} failed, return code is {}"\
                            .format('bin_file', ret_code))


if __name__ == "__main__":
    sys.exit(main(sys.argv))
