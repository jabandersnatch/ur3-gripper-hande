################################################################################

# Set the Docker container name from a project name (first argument).
# If no argument is given, use the current user name as the project name.
$PROJECT = "juana"
$CONTAINER = "juana-ros_ur3-1"
Write-Host "${PSCommandPath}: PROJECT=${PROJECT}"
Write-Host "${PSCommandPath}: CONTAINER=${CONTAINER}"

# Run the Docker container in the background.
# Any changes made to './docker/docker-compose.yml' will recreate and overwrite the container.
docker-compose -p ${PROJECT} -f ./docker-compose.yml up -d

################################################################################

# Display GUI through X Server by granting full access to any external client.
# Note: This step is not required on Windows if you're using VcXsrv or Xming.

################################################################################

# Enter the Docker container with a Bash shell (with or without a custom).
docker exec -it -e DISPLAY=host.docker.internal:0.0 ${CONTAINER} bash

################################################################################

