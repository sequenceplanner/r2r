FROM kristoferb/spdesktop_ros2:rolling

COPY . /r2r
COPY ./github/workflows/test.sh /r2r
RUN rm /bin/sh && ln -s /bin/bash /bin/sh
RUN chmod +x /r2r/test.sh
ENTRYPOINT [ "/r2r/test.sh" ]
