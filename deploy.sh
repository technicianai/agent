#!/bin/bash

echo "Running '$0' for '${MODULE}'"
ACCOUNT=634124784535
ECR_REPOSITORY=${ACCOUNT}.dkr.ecr.${AWS_REGION}.amazonaws.com/${MODULE}
echo "$ECR_REPOSITORY"
echo "${MODULE}":"${VERSION}"
echo "${ECR_REPOSITORY}":"${VERSION}"

docker tag "${MODULE}":"${VERSION}" "${ECR_REPOSITORY}":"${VERSION}"
docker tag "${MODULE}":"${VERSION}" "${ECR_REPOSITORY}":latest
aws ecr get-login-password --region "${AWS_REGION}" | docker login --username AWS --password-stdin "${ECR_REPOSITORY}"
docker push "${ECR_REPOSITORY}":"${VERSION}"
docker push "${ECR_REPOSITORY}":latest
