# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.db import migrations, models


class Migration(migrations.Migration):

    dependencies = [
        ('storeApp', '0003_auto_20151202_2007'),
    ]

    operations = [
        migrations.CreateModel(
            name='Contains',
            fields=[
                ('id', models.AutoField(verbose_name='ID', serialize=False, auto_created=True, primary_key=True)),
                ('stock', models.IntegerField()),
            ],
        ),
        migrations.RenameField(
            model_name='user',
            old_name='user_password',
            new_name='password',
        ),
        migrations.RenameField(
            model_name='user',
            old_name='user_name',
            new_name='username',
        ),
        migrations.RemoveField(
            model_name='order',
            name='user',
        ),
        migrations.RemoveField(
            model_name='product',
            name='order',
        ),
        migrations.RemoveField(
            model_name='supplier',
            name='product',
        ),
        migrations.AddField(
            model_name='order',
            name='orders',
            field=models.ForeignKey(default=1, editable=False, to='storeApp.User'),
        ),
        migrations.AddField(
            model_name='product',
            name='orders',
            field=models.ForeignKey(default=1, editable=False, to='storeApp.Order'),
        ),
        migrations.AddField(
            model_name='product',
            name='supplies',
            field=models.ForeignKey(default=1, editable=False, to='storeApp.Supplier'),
        ),
        migrations.AlterField(
            model_name='user',
            name='user_is_staff',
            field=models.BooleanField(default=True),
        ),
        migrations.AddField(
            model_name='contains',
            name='products',
            field=models.ManyToManyField(to='storeApp.Order', through='storeApp.Product'),
        ),
        migrations.AddField(
            model_name='product',
            name='contains',
            field=models.ForeignKey(default=1, editable=False, to='storeApp.Contains'),
        ),
    ]
