from flask_login import LoginManager, UserMixin
from werkzeug.security import generate_password_hash, check_password_hash
from database_user import get_user_by_username

login_manager = LoginManager()

class User(UserMixin):
    def __init__(self, id, username, password, role):
        self.id = id
        self.username = username
        self.password = password  # Stocké sous forme hachée
        self.role = role

@login_manager.user_loader
def load_user(user_id):
    user = get_user_by_username(user_id)
    if user:
        return User(user["id"], user["username"], user["password"], user["role"])
    return None

def authenticate(username, password):
    user = get_user_by_username(username)
    if user and check_password_hash(user["password"], password):
        return User(user["id"], user["username"], user["password"], user["role"])
    return None
