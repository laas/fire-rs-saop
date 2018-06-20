# Copyright (c) 2018, CNRS-LAAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import subprocess
import abc


class NoUI:
    def message_dialog(self, message: str):
        pass

    def warning_dialog(self, message: str):
        pass

    def error_dialog(self, message: str):
        pass

    def question_dialog(self, message: str) -> bool:
        """Always returns True"""
        return True


class ZenityUI(NoUI):
    def message_dialog(self, message: str):
        """Show a dialog window with a 'message' next an image 'type'"""
        subprocess.run(["zenity", "--info",
                        "--text={}".format(message)])

    def warning_dialog(self, message: str):
        """Show a dialog window with a 'message' next an image 'type'"""
        subprocess.run(["zenity", "--warning",
                        "--text={}".format(message)])

    def error_dialog(self, message: str):
        """Show a dialog window with a 'message' next an image 'type'"""
        subprocess.run(["zenity", "--error",
                        "--text={}".format(message)])

    def question_dialog(self, message: str) -> bool:
        """Show a dialog window with a 'message' next an image 'type'"""
        if subprocess.run(["zenity", "--question",
                           "--text={}".format(message)]).returncode == 0:
            return True
        else:
            return False

