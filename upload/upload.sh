package_name="swerv-iss"
whisper_id="22"
ralph_id="104"
os_name="ubuntu"
release_name=$CI_COMMIT_TAG



curl --header "JOB-TOKEN: $CI_JOB_TOKEN" --upload-file "build-Linux/whisper"  "https://aus-gitlab.local.tenstorrent.com/api/v4/projects/${ralph_id}/packages/generic/${package_name}/${release_name}/whisper"
curl --header  "JOB-TOKEN: $CI_JOB_TOKEN"  --upload-file "build-Linux/whisper"  "https://aus-gitlab.local.tenstorrent.com/api/v4/projects/${whisper_id}/packages/generic/${package_name}/${release_name}/whisper"


