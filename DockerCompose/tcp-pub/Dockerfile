FROM python:latest
WORKDIR "/"
COPY ./ /
RUN pip3 install -r /req.txt
CMD ["python3", "/publisher.py"]