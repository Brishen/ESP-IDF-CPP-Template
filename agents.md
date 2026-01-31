# Build Instructions for Agents

This document outlines the steps required for an AI agent to build this project.

## Prerequisites

The shell environment must have the ESP-IDF tools available.

## Build Steps

1.  **Initialize Environment**
    Run the following alias to set up the ESP-IDF environment variables:
    ```bash
    get_idf
    ```

2.  **Update Submodules (Critical)**
    Ensure all ESP-IDF components are up to date.
    ```bash
    cd $IDF_PATH
    git submodule update --init --recursive
    cd -
    ```

2.  **Build Project**
    Execute the build command:
    ```bash
    idf.py build
    ```
