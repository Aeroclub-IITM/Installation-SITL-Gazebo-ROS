#!/bin/bash


set -e

spatialPrint() {
    echo ""
    echo ""
    echo "$1"
	echo "================================"
}

# To note: the execute() function doesn't handle pipes well
execute () {
	echo "$ $*"
	OUTPUT=$($@ 2>&1)
	if [ $? -ne 0 ]; then
        echo "$OUTPUT"
        echo ""
        echo "Failed to Execute $*" >&2
        exit 1
    fi
}

# Speed up the process
# Env Var NUMJOBS overrides automatic detection
if [[ -n $NUMJOBS ]]; then
    MJOBS=$NUMJOBS
elif [[ -f /proc/cpuinfo ]]; then
    MJOBS=$(grep -c processor /proc/cpuinfo)
elif [[ "$OSTYPE" == "darwin"* ]]; then
	MJOBS=$(sysctl -n machdep.cpu.thread_count)
else
    MJOBS=4
fi

execute sudo apt-get update -y
execute sudo apt-get install build-essential curl g++ cmake cmake-curses-gui git pkg-config checkinstall -y
execute sudo apt-get install libopenblas-dev liblapacke-dev libatlas-base-dev gfortran -y

# Choice for terminal that will be adopted: Tilda+tmux
# Not guake because tilda is lighter on resources
# Not terminator because tmux sessions continue to run if you accidentally close the terminal emulator
execute sudo apt-get install git wget curl -y
execute sudo apt-get install tilda tmux -y
execute sudo apt-get install gimp -y
execute sudo apt-get install xclip -y # this is used for the copying tmux buffer to clipboard buffer
execute sudo apt-get install vim-gui-common vim-runtime -y
cp ./config_files/vimrc ~/.vimrc
execute sudo snap install micro --classic
mkdir -p ~/.config/micro/
cp ./config_files/micro_bindings.json ~/.config/micro/bindings.json

# refer : [http://www.rushiagr.com/blog/2016/06/16/everything-you-need-to-know-about-tmux-copy-pasting-ubuntu/] for tmux buffers in ubuntu
cp ./config_files/tmux.conf ~/.tmux.conf
cp ./config_files/tmux.conf.local ~/.tmux.conf.local
mkdir -p ~/.config/tilda
cp ./config_files/config_0 ~/.config/tilda/

#Checks if ZSH is partially or completely Installed to Remove the folders and reinstall it
rm -rf ~/.z*
zsh_folder=/opt/.zsh/
if [[ -d $zsh_folder ]];then
	sudo rm -r /opt/.zsh/*
fi

spatialPrint "Setting up Zsh + Zim now"
execute sudo apt-get install zsh -y
sudo mkdir -p /opt/.zsh/ && sudo chmod ugo+w /opt/.zsh/
git clone --recursive --quiet --branch zsh-5.2 https://github.com/zimfw/zimfw.git /opt/.zsh/zim
ln -s /opt/.zsh/zim/ ~/.zim
ln -s /opt/.zsh/zim/templates/zimrc ~/.zimrc
ln -s /opt/.zsh/zim/templates/zlogin ~/.zlogin
ln -s /opt/.zsh/zim/templates/zshrc ~/.zshrc
git clone https://github.com/zsh-users/zsh-autosuggestions /opt/.zsh/zsh-autosuggestions
echo "source /opt/.zsh/zsh-autosuggestions/zsh-autosuggestions.zsh" >> ~/.zshrc
# Change default shell to zsh
command -v zsh | sudo tee -a /etc/shells
sudo chsh -s "$(command -v zsh)" "${USER}"

execute sudo apt-get install aria2 -y

# Create bash aliases
cp ./config_files/bash_aliases /opt/.zsh/bash_aliases
ln -s /opt/.zsh/bash_aliases ~/.bash_aliases

{
    echo "if [ -f ~/.bash_aliases ]; then"
    echo "  source ~/.bash_aliases"
    echo "fi"

    echo "# Switching to 256-bit colour by default so that zsh-autosuggestion's suggestions are not suggested in white, but in grey instead"
    echo "export TERM=xterm-256color"

    echo "# Setting the default text editor to micro, a terminal text editor with shortcuts similar to what you'd encounter in an IDE"
    echo "export VISUAL=micro"
} >> ~/.zshrc

# Now create shortcuts
execute sudo apt-get install run-one xbindkeys xbindkeys-config wmctrl xdotool -y
cp ./config_files/xbindkeysrc ~/.xbindkeysrc

# Check if Anaconda is already installed
if [[ -n $(echo $PATH | grep 'conda') ]]; then
    echo "Anaconda is already installed, skipping installation"
    echo "To reinstall, delete the Anaconda install directory and remove from PATH as well"
else

    spatialPrint "Installing the latest Anaconda Python in /opt/anaconda3"
    continuum_website=https://repo.continuum.io/archive/
    # Stepwise filtering of the html at $continuum_website
    # Get the topmost line that matches our requirements, extract the file name.
    latest_anaconda_setup=$(wget -q -O - $continuum_website index.html | grep "Anaconda3-" | grep "Linux" | grep "86_64" | head -n 1 | cut -d \" -f 2)
    aria2c --file-allocation=none -c -x 10 -s 10 -o anacondaInstallScript.sh --dir ./extras ${continuum_website}${latest_anaconda_setup}
    sudo mkdir -p /opt/anaconda3 && sudo chmod ugo+w /opt/anaconda3
    execute bash ./extras/anacondaInstallScript.sh -f -b -p /opt/anaconda3

    spatialPrint "Setting up your anaconda"
    execute /opt/anaconda3/bin/conda update conda -y
    execute /opt/anaconda3/bin/conda clean --all -y
    execute /opt/anaconda3/bin/conda install ipython -y

    execute /opt/anaconda3/bin/conda install libgcc -y
    execute /opt/anaconda3/bin/pip install numpy scipy matplotlib scikit-learn scikit-image jupyter notebook pandas h5py cython jupyterlab
    execute /opt/anaconda3/bin/pip install msgpack
    execute /opt/anaconda3/bin/conda install line_profiler -y
    sed -i.bak "/anaconda3/d" ~/.zshrc

    /opt/anaconda3/bin/conda info -a

    spatialPrint "Adding anaconda to path variables"
    {
        echo "# Anaconda Python. Change the \"conda activate base\" to whichever environment you would like to activate by default"
        echo ". /opt/anaconda3/etc/profile.d/conda.sh"
        echo "conda activate base"
    } >> ~/.zshrc

fi # Anaconda Installation end

# echo "*************************** NOTE *******************************"
# echo "If you ever mess up your anaconda installation somehow, do"
# echo "\$ conda remove anaconda matplotlib mkl mkl-service nomkl openblas"
# echo "\$ conda clean --all"
# echo "Do this for each environment as well as your root. Then reinstall all except nomkl"

## Detect if an Nvidia card is attached, and install the graphics drivers automatically
# if [[ -n $(lspci | grep -i nvidia) ]]; then
#     spatialPrint "Installing Display drivers and any other auto-detected drivers for your hardware"
#     execute sudo add-apt-repository ppa:graphics-drivers/ppa -y
#     execute sudo apt-get update
#     execute sudo ubuntu-drivers autoinstall
# fi

# Install code editor of your choice
if [[ ! -n $CIINSTALL ]]; then
    read -p "Download and Install VS Code / Atom / Sublime. Press q to skip this. Default: Skip Editor installation [v/a/s/q]: " tempvar
fi
tempvar=${tempvar:-q}

if [ "$tempvar" = "v" ]; then
    curl https://packages.microsoft.com/keys/microsoft.asc | gpg --dearmor > microsoft.gpg
    sudo install -o root -g root -m 644 microsoft.gpg /etc/apt/trusted.gpg.d/
    sudo sh -c 'echo "deb [arch=amd64] https://packages.microsoft.com/repos/vscode stable main" > /etc/apt/sources.list.d/vscode.list'
    execute sudo apt-get install apt-transport-https -y
    execute sudo apt-get update
    execute sudo apt-get install code -y # or code-insiders
    execute rm microsoft.gpg
elif [ "$tempvar" = "a" ]; then
    execute sudo add-apt-repository ppa:webupd8team/atom
    execute sudo apt-get update; execute sudo apt-get install atom -y
elif [ "$tempvar" = "s" ]; then
    wget -q -O - https://download.sublimetext.com/sublimehq-pub.gpg | sudo apt-key add -
    echo "deb https://download.sublimetext.com/ apt/stable/" | sudo tee /etc/apt/sources.list.d/sublime-text.list
    execute sudo apt-get install apt-transport-https -y
    execute sudo apt-get update
    execute sudo apt-get install sublime-text -y
elif [ "$tempvar" = "q" ];then
    echo "Skipping this step"
fi

# Browsers
wget -q -O - https://dl-ssl.google.com/linux/linux_signing_key.pub | sudo apt-key add - 
sudo sh -c 'echo "deb [arch=amd64] http://dl.google.com/linux/chrome/deb/ stable main" >> /etc/apt/sources.list.d/google-chrome.list'
execute sudo apt-get update  -y
execute sudo apt-get install google-chrome-stable -y
#execute sudo apt-get install chromium-browser -y
execute sudo apt-get install firefox -y

