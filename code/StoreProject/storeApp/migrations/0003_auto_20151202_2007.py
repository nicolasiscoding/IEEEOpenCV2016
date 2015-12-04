# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models
import django.core.validators


class Migration(migrations.Migration):

    dependencies = [
        ('storeApp', '0002_auto_20151201_0050'),
    ]

    operations = [
        migrations.AlterField(
            model_name='user',
            name='user_password',
            field=models.CharField(max_length=20, validators=[django.core.validators.MinLengthValidator(8, b'Your password must contain at least 8 characters.')]),
        ),
    ]
