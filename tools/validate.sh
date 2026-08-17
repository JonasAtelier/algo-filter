#!/bin/sh

set -eu

usage()
{
	printf '%s\n' \
		"Usage: $0 [--help]" \
		"" \
		"Build and run the strict host tests. If arduino-cli and the" \
		"ESP32 core are installed, also compile both Arduino examples."
}

if [ "$#" -eq 1 ] && { [ "$1" = "-h" ] || [ "$1" = "--help" ]; }; then
	usage
	exit 0
fi

if [ "$#" -ne 0 ]; then
	usage >&2
	exit 2
fi

script_dir=$(CDPATH= cd "$(dirname "$0")" && pwd)
repo_root=$(CDPATH= cd "$script_dir/.." && pwd)
arduino_build_dir=

cleanup()
{
	status=$?

	if ! make -f tests/Makefile clean; then
		if [ "$status" -eq 0 ]; then
			status=1
		fi
	fi

	if [ -n "$arduino_build_dir" ] &&
	   ! rm -rf "$arduino_build_dir"; then
		if [ "$status" -eq 0 ]; then
			status=1
		fi
	fi

	trap - 0
	exit "$status"
}

arduino_core_available()
{
	core_list=$(arduino-cli core list 2>/dev/null) || return 1

	printf '%s\n' "$core_list" |
		awk '$1 == "esp32:esp32" { found = 1 }
		     END { exit !found }'
}

compile_example()
{
	example_name=$1

	printf 'Compiling Arduino example: %s\n' "$example_name"
	arduino-cli compile \
		--fqbn esp32:esp32:esp32 \
		--library "$repo_root" \
		--build-path "$arduino_build_dir/$example_name" \
		"$repo_root/examples/$example_name"
}

cd "$repo_root"
trap cleanup 0
trap 'exit 1' 1 2 3 15

printf '%s\n' "Running host tests"
make -f tests/Makefile clean
make -f tests/Makefile test

if ! command -v arduino-cli >/dev/null 2>&1; then
	printf '%s\n' \
		"Skipping Arduino examples: arduino-cli is not installed."
elif ! arduino_core_available; then
	printf '%s\n' \
		"Skipping Arduino examples: ESP32 core esp32:esp32 is unavailable."
else
	arduino_build_dir=$(mktemp -d \
		"${TMPDIR:-/tmp}/esp32-kalman-filter.XXXXXX")
	compile_example basic_smoothing
	compile_example custom_tuning
fi
