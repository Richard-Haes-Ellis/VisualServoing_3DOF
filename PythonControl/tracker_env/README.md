# Install the virtualenv package

The virtualenv package is required to create virtual environments. You can install it with pip:

    pip install virtualenv

# Create the virtual environment

To create a virtual environment, you must specify a path. For example to create one in the local directory called ‘mypython’, type the following:

    virtualenv mypython

# Activate the virtual environment

You can activate the python environment by running the following command:
Mac OS / Linux

    source mypython/bin/activate

Windows

    mypthon\Scripts\activate

You should see the name of your virtual environment in brackets on your terminal line e.g. (mypython).

Any python commands you use will now work with your virtual environment

# Requirements Files

“Requirements files” are files containing a list of items to be installed using pip install like so:

    pip install -r requirements.txt

