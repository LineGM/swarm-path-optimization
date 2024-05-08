#!/bin/bash

EXE="@CMAKE_CURRENT_BINARY_DIR@/cmake-init-cpp-gui.exe"

search_paths=("/usr/@CMAKE_COMPILER_PREFIX@/bin"
             #"/usr/lib/gcc/@CMAKE_COMPILER_PREFIX@/13.1.0/"
              "/usr/@CMAKE_COMPILER_PREFIX@/lib")

found_dlls=()
not_found_dlls=()

function findAndCopyDLL() {
    for i in "${search_paths[@]}"
    do
        FILE="$i/$1"
        if [ -f $FILE ]; then
           cp $FILE @CMAKE_BINARY_DIR@
           found_dlls+=($1)
           copyForOBJ $FILE
           return 0
        fi
    done

    return 1
}

function copyForOBJ() {
    dlls=`@CMAKE_COMPILER_PREFIX@-objdump -p $1 | grep 'DLL Name:' | sed -e "s/\t*DLL Name: //g"`
    while read -r filename; do
        findAndCopyDLL $filename || not_found_dlls+=($filename)
    done <<< "$dlls"
}

function printUniqs() {
    echo "Found dlls:"
    uniqs_found=($(for dll in "${found_dlls[@]}"; do echo "${dll}"; done | sort -u))
    for i in "${uniqs_found[@]}"
    do
        echo "$i"
    done

    echo ""

    echo "Not found dlls:"
    uniqs_not_found=($(for dll in "${not_found_dlls[@]}"; do echo "${dll}"; done | sort -u))
    for i in "${uniqs_not_found[@]}"
    do
        echo "$i"
    done

    return 0
}

# Copy DLLs with objdump
copyForOBJ $EXE
# Print founds and not founds
printUniqs
# Copy ...qt/plugins/platforms/
cp -r "@qt_plugins_path@" "@CMAKE_CURRENT_BINARY_DIR@"
