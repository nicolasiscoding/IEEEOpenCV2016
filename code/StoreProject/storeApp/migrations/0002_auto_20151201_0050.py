# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('storeApp', '0001_initial'),
    ]

    operations = [
        migrations.AddField(
            model_name='user',
            name='user_firstname',
            field=models.CharField(default='First Name', max_length=50),
            preserve_default=False,
        ),
        migrations.AddField(
            model_name='user',
            name='user_lastname',
            field=models.CharField(default='Last Name', max_length=50),
            preserve_default=False,
        ),
    ]
