FROM python:3.8.12

COPY pyrobosim /pyrobosim
COPY test test/
COPY setup.py setup.py