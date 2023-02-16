#!/bin/bash
set -u # Fail if environment vars not set
: "$AWS_REGION"
: "$MODULE"
: "$VERSION"
: "$ACCOUNT_ID"

echo "Running '$0' for '${MODULE}'"
ECR_REPOSITORY=${ACCOUNT_ID}.dkr.ecr.${AWS_REGION}.amazonaws.com/${MODULE}
echo "$ECR_REPOSITORY"
echo "${MODULE}":"${VERSION}"
echo "${ECR_REPOSITORY}":"${VERSION}"

docker tag "${MODULE}":"${VERSION}" "${ECR_REPOSITORY}":"${VERSION}"
docker tag "${ECR_REPOSITORY}":"${VERSION}" "${ECR_REPOSITORY}":latest
aws ecr get-login-password --region "${AWS_REGION}" | docker login --username AWS --password-stdin "${ECR_REPOSITORY}"
exit 1
docker push "${ECR_REPOSITORY}":"${VERSION}"
docker push "${ECR_REPOSITORY}":latest
