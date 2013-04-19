#!/bin/sh

#!/bin/sh
sourcedir=$(dirname $(readlink -f $0))
modulename=nrf24l01
git submodule add "$sourcedir" "$modulename"
echo "include $modulename/Makefile.module" >> Makefile.modules
