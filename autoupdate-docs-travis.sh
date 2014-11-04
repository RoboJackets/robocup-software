#!/bin/bash

set -e

# This script is called by .travis.yml to update our doxygen documentation, which is stored on the gh-pages branch of this repo
# It's based on this blog post: http://philipuren.com/serendipity/index.php?/archives/21-Using-Travis-to-automatically-publish-documentation.html



if [ "$(git rev-parse master)" = "$TRAVIS_COMMIT" ]; then
    # build doxygen from source so we can have the latest and greatest
    sudo apt-get install flex bison 
    git clone https://github.com/doxygen/doxygen.git
    cd doxygen
    ./configure
    make
    sudo make install
    doxygen --version
    cd ..

    sudo apt-get install graphviz

    git config --global user.name "$GIT_USERNAME"
    git config --global user.email $GIT_EMAIL
    mkdir -p api_docs
    git clone -b gh-pages git://github.com/robojackets/robocup-software api_docs/html
    doxygen
    cp doxygen.css api_docs/html
    cd api_docs/html
    git add --all
    git commit -m 'auto-updated api docs'
    git push https://$GH_TOKEN@github.com/robojackets/robocup-software gh-pages
fi
