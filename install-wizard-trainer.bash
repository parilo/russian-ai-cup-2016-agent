#!/bin/bash +

BASEDIR=`pwd`
INSTALLDIR=${BASEDIR}/installdir
BUILDDIR=${BASEDIR}/builddir

export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${INSTALLDIR}/lib"
export PATH="${PATH}:${INSTALLDIR}/bin"


echo "installing Wizard Trainer"
BUILDDIR=${BUILDDIR}/build-wt
mkdir ${BUILDDIR}
cd ${BUILDDIR}
cmake -DCMAKE_INSTALL_PREFIX=${INSTALLDIR} -DTF_GRAPH_DIR=${BASEDIR}/TensorflowGraph/models/ -DTF_GRAPH_SAVE_DIR=${BASEDIR}/TensorflowGraph/states/ ${BASEDIR}/WizardTrainer/WizardTrainer
make
make install
cd ${BASEDIR}
