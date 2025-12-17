#! /bin/bash -eux

echo "Running, $CI_REPO_URL, $CI_WORKSPACE"

PRJ_NAME="meter_reader"

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

cd $SCRIPT_DIR/..
pwd
ls

export AR_PROJECTS_DIR="$CI_WORKSPACE"
export AR_LIB_DIR="$CI_WORKSPACE/kicadstuff/"

$CI_WORKSPACE/kicadstuff/scripts/build_artifacts.sh "$PRJ_NAME.kicad_sch" "$PRJ_NAME.kicad_pcb" "mfgout"

find mfgout/

zip -r mfgout.zip mfgout

curl -v --user alex:${GITEA_TOKEN} --upload-file mfgout.zip https://gitea.home.svcs.alexroth.me/api/packages/alex/generic/$PRJ_NAME/${CI_COMMIT_SHA:-5}/mfgout.zip

