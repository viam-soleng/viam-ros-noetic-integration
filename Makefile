setup_venv:
	git submodule update
	python3 -m venv venv
	. ./venv/bin/activate; pip install -r ./requirements.txt; deactivate

build_services: setup_venv
	. ./venv/bin/activate; \
	protoc -I./proto -I./proto/googleapis --python_out=./proto --grpclib_python_out=./proto ./proto/noetic_*.proto; \
	deactivate