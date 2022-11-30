echo "launching ER-Force framework and our UI"
(trap 'kill 0' SIGINT; find ~ -name 'simulator-cli' -type f -exec '{}' -g 2020B ';' & make run-sim & wait)