FROM python:3.8

## preliminary needed only for being compatible with M1 architecture
RUN apt update && apt upgrade -y
RUN apt install libgeos-dev python3-gdal libgdal-dev libspatialindex-dev libspatialindex-c6 -y

##install dependencies
WORKDIR /pdm4ar

COPY requirements.txt .
RUN pip install -r requirements.txt

COPY . .
RUN find .
ENV DISABLE_CONTRACTS=1

RUN python setup.py develop --no-deps
RUN pdm4ar-exercise --help

CMD ["bash"]
