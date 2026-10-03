#!/bin/sh
set -eu

app=${1:-./bladegps}

if ! "$app" -L >/dev/null 2>&1; then
	echo "profile listing unexpectedly failed" >&2
	exit 1
fi

expect_rejected()
{
	if "$app" "$@" >/dev/null 2>&1; then
		echo "invalid CLI input was accepted: $*" >&2
		exit 1
	fi
}

expect_rejected -e brdc1700.16n -l nan,0,0 -d 1
expect_rejected -e brdc1700.16n -l 0,0,nan -d 1
expect_rejected -e brdc1700.16n -l 0,0,0 -d nan
expect_rejected -e brdc1700.16n -l 0,0,0 -d 0.01
expect_rejected -e brdc1700.16n -l 0,0,0 -t 2020/01/01,00:00:nan -d 1
expect_rejected -e brdc1700.16n -l 0,0,0 -r 2600001 -d 1
expect_rejected -e brdc1700.16n -l 0,0,0 -G 0 -a -25 -A 0 -d 1

echo "CLI validation tests passed"
