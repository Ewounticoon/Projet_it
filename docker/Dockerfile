# Utiliser l'image officielle de ROS Noetic
FROM ros:noetic

# Mettre à jour les paquets et installer les dépendances
RUN apt-get update && apt-get install -y \
    nano \
    openssh-client \
    git \
    python3-pip \
    sqlite3 \
    libsqlite3-dev \
    && rm -rf /var/lib/apt/lists/*

# Installer les bibliothèques Python nécessaires
RUN pip3 install --no-cache-dir \
    pi-rc522 \
    flask \
    flask-login

# Définir le répertoire de travail dans le conteneur
WORKDIR /root

# Exposer le port Flask
EXPOSE 5000

# Copier le script entrypoint dans le conteneur
COPY ./entrypoint.sh /root/Projet_it/docker/entrypoint.sh
RUN chown -R 1000:1000 /root/Projet_it
RUN chmod +x /root/Projet_it/docker/entrypoint.sh

# Définir l'entrypoint pour exécuter automatiquement le script au démarrage
ENTRYPOINT ["/root/Projet_it/docker/entrypoint.sh"]

# Commande pour garder le conteneur actif (ex: bash ou lancement automatique d'un script)
CMD ["/bin/bash"]