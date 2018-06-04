/* Copyright (c) 2017-2018, CNRS-LAAS
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

 * Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

 * Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */
#include "saop_logging.hpp"

void SAOP::PythonLoggerSink::consume(logging::record_view const& rec) {
    /*
     * a_rec = logger.makeRecord(logger.name, logging.CRITICAL, "module", 123, "msg %s", (logger.name,), ())
     * logger.handle(a_rec)
     * Logger.makeRecord(name, lvl, fn, lno, msg, args, exc_info, func=None, extra=None, sinfo=None)
     */
    logging::trivial::severity_level boost_severity = static_cast<logging::trivial::severity_level>(rec[logging::trivial::severity].get());
    PythonLoggingLevels py_severity = PythonLoggingLevels::notset;
    switch (boost_severity) {
        case logging::trivial::severity_level::trace:
            py_severity = PythonLoggingLevels::notset;
            break;
        case logging::trivial::severity_level::debug:
            py_severity = PythonLoggingLevels::debug;
            break;
        case logging::trivial::severity_level::info:
            py_severity = PythonLoggingLevels::info;
            break;
        case logging::trivial::severity_level::warning:
            py_severity = PythonLoggingLevels::warning;
            break;
        case logging::trivial::severity_level::error:
            py_severity = PythonLoggingLevels::error;
            break;
        case logging::trivial::severity_level::fatal:
            py_severity = PythonLoggingLevels::critical;
            break;
        default:
            break;
    }
    {
        // Python objects must be called with the GIL locked. If not, object's attr may be unavailable misteriously
        py::gil_scoped_acquire acquire;
        // TODO: Register current line with boost::log so the record can be complete
        auto py_record = logger.attr("makeRecord")(module_name, static_cast<int>(py_severity), module_name, 0,
                                                   py::str(rec[expr::smessage].get()), py::none(), py::none());
        logger.attr("handle")(py_record);
        //logger.attr("log")(static_cast<int>(py_severity), py::str(rec[expr::smessage].get()));
    }
}