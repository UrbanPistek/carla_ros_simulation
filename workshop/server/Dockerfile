FROM python:3.7-alpine

WORKDIR /usr/src/app

RUN pip install flask

COPY ./server.py .

CMD [ "python", "-u", "./server.py" ]

