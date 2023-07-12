# Contributing to pyrobosim

Thank you for considering a contribution to `pyrobosim`!

While jumping into someone else's code base can be challenging, here are some tips to help you navigate contributing to this repository.

## Getting Started

Before you can contribute, you should first make sure you can run `pyrobosim`.
Refer to the [setup documentation](https://pyrobosim.readthedocs.io/en/latest/setup.html), and choose either a local or Docker based installation.

## Submitting an Issue

If you have any issues with `pyrobosim`, or any ideas to make the tool better for yourself and others, please [submit a Git issue](https://github.com/sea-bass/pyrobosim/issues/new)!

Make sure to take a look at the [list of open issues](https://github.com/sea-bass/pyrobosim/issues) first.

## Making Contributions

Before submitting a pull request (PR) to this repository, there are a few tools you can use to increase the chance that your changes will be accepted.

### Formatting

We use [pre-commit](https://pre-commit.com/) for code formatting.
These checks must pass for a PR to be merged.

To set up pre-commit locally on your system:

* Install pre-commit: `pip3 install pre-commit`.
* Go to the root folder of this repository.
* Run `pre-commit run -a` before committing changes.
* Commit any of the changes that were automatically made, or fix them otherwise.

NOTE: You can also run `pre-commit install` to automatically run pre-commit before your commits.

### Tests

You can run all unit tests locally before you push to your branch and wait for CI to complete.
To do this:

* Go to the root folder of this repository.
* Run `test/run_tests.bash` (or `test/run_tests.bash $ROS_DISTRO` if using ROS)

The above script runs [`pytest`](https://docs.pytest.org/) with the settings configured for this test.
You can also look at, and modify, the `pytest.ini` file at the root of this repository.

If you are using a Docker based workflow, you can also run the tests inside a container:

```
docker compose run test
```

In both cases, the latest test results will be saved to the `test/results` folder.
You can open the `test_results.html` file using your favorite browser if you want to dig into more details about your test results.

### Documentation

You can also build the `pyrobosim` documentation locally.
This lets you verify how changes to the documentation (new pages, changes to existing pages, docstrings in the Python code, etc.) look before you push to your branch.

To build docs locally:

* Go to the root folder of this repository.
* Run `setup/generate_docs.bash`

This will generate docs pages to the `docs/build` folder.
You can view the homepage by opening `docs/build/html/index.html` using your favorite browser.

The docs are built using [Sphinx](https://www.sphinx-doc.org/en/master/) and [ReadTheDocs (RTD)](https://about.readthedocs.com/).
As such, you may need to install additional Python packages for the build to succeed.

```
pip3 install sphinx sphinx-rtd-theme
```
