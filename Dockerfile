FROM golang:1.13-alpine AS builder

WORKDIR $GOPATH/src/github.com/chriscoffee/go-pi2go

COPY . .

ARG VERSION="unset"

RUN DATE="$(date -u +%Y-%m-%d-%H:%M:%S-%Z)" \
    && GO111MODULE=on CGO_ENABLED=0 GOPROXY="https://proxy.golang.org" \
    go build -ldflags "-s -w -X github.com/chriscoffee/go-pi2go/cmd.Version=$VERSION -X github.com/chriscoffee/go-pi2go/cmd.Build=$DATE" -o /bin/go-pi2go .

FROM alpine

ENV GO111MODULE=on

COPY --from=builder /bin/go-pi2go /bin/go-pi2go
COPY --from=builder /usr/local/go/bin/go /bin/go

ENV GO_ENV=production

ENTRYPOINT ["go-pi2go"]
