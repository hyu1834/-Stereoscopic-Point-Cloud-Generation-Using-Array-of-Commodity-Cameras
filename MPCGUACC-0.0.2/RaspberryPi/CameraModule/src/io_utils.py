## @file io_utils.py
## @package io_utils
# @author Hiu Hong Yu (AHMCT)
# @brief Module contains functions for in and out stream.
# @details
#  Methods including:\n
#  <ul>
#  <li>output usage</li>
#  <li>output error</li>
#  <li>output logs</li>
#  </ul>
#

import sys
import os

## @brief standard usage out stream.
# @param message - message to output as usage
# @param terminate = False - terminate program after displaying message (default = False)
# @return None
def usage(message, terminate = False):
	sys.stderr.write("USAGE: %s\n" %(message))

	# if should terminate the program
	if terminate:
		sys.exit(1)

## @brief standard error out stream.
# @param message - message to output as error
# @param terminate = False - terminate program after displaying message (default = False)
# @return None
def stderr(message, terminate = False):
	sys.stderr.write('Error: %s\n'%(message))
	
	# if should terminate the program
	if terminate:
		sys.exit(1)

## @brief function error out stream.
# error output with location provided
# @param message - message to output as 
# @param function_name - name of function throwing error
# @param terminate = False - terminate program after displaying message (default = False)
# @return None
def function_stderr(message,function_name, terminate = False):
	sys.stderr.write('Error: %s in %s\n'%(message,function_name))

	# if should terminate the program
	if terminate:
		sys.exit(1)


## @brief standard warning out stream
# @param message - message to output as warning
# @param terminate = False - terminate program after displaying message (default = False)
# @return None
def warning(message, terminate = False):
	sys.stderr.write("WARNING: %s\n" %(message))

	# if should terminate the program
	if terminate:
		sys.exit(1)

## @brief function warning out stream
# warning out stream with location provided
# @param message - message to output as warning
# @param function_name - name of function throwing warning
# @param terminate = False - terminate program after displaying message (default = False)
# @return None
def function_warning(message,function_name, terminate = False):
	sys.stderr.write("WARNING: %s in %s\n" %(message,function_name))

	# if should terminate the program
	if terminate:
		sys.exit(1)


## @brief standard log out stream
# @param message - message to output as logs
# @return None
def stdlog(message):
	sys.stderr.write('%s\n'%(message))

## @brief status updater
# @param message - message to output as status
# @return None
def print_progress_status(message):
    sys.stderr.write('\r' + message)
    sys.stderr.flush()

