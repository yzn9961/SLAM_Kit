$ cat bin/cntline.sh
#!/bin/bash

ALL_LINE=0
ALL_FILE=0
FILES=""
# 文件类型过滤器
# FILTER="\.cpp$|\.c$|\.html|\.h$|\.go$|\.py$|\.sh$"
FILTER="\.cpp$|\.c$|\.h$|\.cc$"
CVRT_FILTER=""
BE_QUITE_DIR=1
BE_QUITE_FILE=1
SHOW_HELP=0
ONLY_SHOW_DIR=0

function showHelp()
{
    MY_NAME=`basename ${0}`
    echo "Usage : ${MY_NAME} -[qQhd]"
    echo -e "\t-q : not show dir"
    echo -e "\t-Q : not show file name"
    echo -e "\t-h : show this help message"
    echo -e "\t-d : only show dirs"
}

set -- `getopt -q v:qQhd "$@"`
#echo -e "grev : $@"
while [ -n "$1" ]
do
    case "$1" in
        -v)
        CVRT_FILTER="${CVRT_FILTER} $2"
        shift
        ;;
        -q)
        BE_QUITE_DIR=0
        shift
        ;;
        -Q)
        BE_QUITE_DIR=0
        BE_QUITE_FILE=0
        shift
        ;;
        -h)
        SHOW_HELP=1
        shift
        ;;
        -d)
        ONLY_SHOW_DIR=1
        shift
        ;;
    esac
    shift
done

function showGrade()
{
    if [ $# -gt 0 ] && [ $1 -gt 0 ]; then
        for ((i = 0; i < $1; i++)) ; do
            echo -n "   "
        done
    fi
}

function a_count_dir()
{
    local file_list=`ls -Sr . | grep -E ${FILTER}`
    local dir_list=`ls .`
    local CURT_DIR=`pwd`

    if ! [ -z "${file_list}" ];then
        if [ ${BE_QUITE_DIR} -eq 1 ];then
            echo -e "${CURT_DIR}"
        fi
    fi
    # scan files
    for x in ${file_list} ; do
        if [ -f ${x} ]; then
            local count_line=`cat ${x} | wc -l`
            ALL_LINE=$[ ${ALL_LINE} + ${count_line} ]
            ALL_FILE=$[ ${ALL_FILE} + 1 ]
            if [ ${BE_QUITE_FILE} -eq 1 ];then
                printf "%5d : %s\n" ${count_line} ${x}
            fi
        else
            continue
        fi
    done

    # scan dirs
    for x in ${dir_list} ;do
        if [ -f ${x} ]; then
            continue
        else
            local NOE_DIR=`pwd`
            cd ${x}
            a_count_dir $[ ${1} + 1 ]
            cd ${NOE_DIR}
        fi
    done
}

function count_dir()
{
    a_count_dir 0
    echo "all line : ${ALL_LINE}"
    echo "all file : ${ALL_FILE}"
}

function showDir()
{
    local CURRENT_DIR=`pwd`
    echo -e "${CURRENT_DIR}"
    local list=`ls`
    for x in ${list}; do
        if [ -f ${x} ];then
            continue
        else
            local NOW_DIR=${CURRENT_DIR}
            cd ${x}
            showDir
            cd ${NOW_DIR}
        fi
    done
}

if [ ${SHOW_HELP} -eq 1 ];then
    showHelp
    exit 0
fi

if [ ${ONLY_SHOW_DIR} -eq 1 ];then
    showDir
    exit 0
fi

count_dir
