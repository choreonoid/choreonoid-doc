#!/bin/bash
rsync -rv ../choreonoid-website/_build/html/en/documents/latest/ cnoidsrv:/var/www/choreonoid.org/public/en/documents/latest/
